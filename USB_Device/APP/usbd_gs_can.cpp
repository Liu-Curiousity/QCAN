#include "usbd_gs_can.h"

#include <string.h>
#include <usbd_core.h>

#include "can_common.h"
#include "timer.h"
#include "led.h"

extern USBD_GS_CAN_HandleTypeDef hGS_CAN;

volatile bool is_usb_suspend_cb;

static can_data_t* USBD_GS_CAN_GetChannel(USBD_GS_CAN_HandleTypeDef *hcan, const uint16_t ch) {
	return ch < ARRAY_SIZE(hcan->channels) ? &hcan->channels[ch] : NULL;
}

// Prefill the buffers from RX from host.
// Return true if pre-allocated buffer list buffer list
// was changed and is now ready, meaning and RX can be
// re-enabled, false otherwise.
// Must be called with IRQ disabled.
static bool USBD_GS_Prefill_RX_Buffers(USBD_GS_CAN_HandleTypeDef *hcan)
{
	unsigned i;
	bool changed;

	changed = false;
	for (i = 0; i < ARRAY_SIZE(hcan->from_host_buf); ++i) {
		if (hcan->from_host_buf[i])
			continue;

		// 从共享池获取空闲的帧对象，用于接收主机发送的数据帧
		hcan->from_host_buf[i] =
			list_first_entry_or_null(&hcan->list_frame_pool, struct gs_host_frame_object, list);
		if (!hcan->from_host_buf[i])
			return false;

		list_del(&hcan->from_host_buf[i]->list);

		changed = true;
	}

	return changed;
}

/*
 * It's unclear from the documentation, but it appears that the USB library is
 * not safely reentrant. It attempts to signal errors via return values if it is
 * reentered, but that code is not interrupt-safe and the error values are
 * silently ignored within the library in several cases. We'll just disable
 * interrupts at all entry points to be safe. Note that the callbacks are all
 * called from within the library itself, either within the interrupt handler or
 * within other calls, which means the USB interrupt is already disabled and we
 * don't have any other interrupts to worry about.
 */
static inline uint8_t USBD_GS_CAN_PrepareReceive(USBD_HandleTypeDef *pdev)
{
	struct gs_host_frame *frame = &hGS_CAN.from_host_buf[0]->frame;
	uint16_t size;

	if (IS_ENABLED(CONFIG_CANFD)) {
		size = struct_size(frame, canfd_ts, 1);
	} else {
		size = struct_size(frame, classic_can_ts, 1);
	}

	return USBD_LL_PrepareReceive(pdev, GSUSB_ENDPOINT_OUT, (uint8_t *)frame, size);
}

static void usbd_gs_can_purge_to_host_buf(USBD_GS_CAN_HandleTypeDef *hcan)
{
	bool was_irq_enabled = disable_irq();

	if (hcan->to_host_buf) {
		list_add_tail(&hcan->to_host_buf->list, &hcan->list_frame_pool);
		hcan->to_host_buf = NULL;
	}

	restore_irq(was_irq_enabled);
}

// device info
static const struct gs_device_config USBD_GS_CAN_dconf = {
	.icount = NUM_CAN_CHANNEL - 1,
	.sw_version = 2,
	.hw_version = 1,
};

static uint8_t USBD_GS_CAN_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
	USBD_LL_OpenEP(pdev, GSUSB_ENDPOINT_IN,	 USBD_EP_TYPE_BULK, CAN_DATA_MAX_PACKET_SIZE);
	USBD_LL_OpenEP(pdev, GSUSB_ENDPOINT_OUT, USBD_EP_TYPE_BULK, CAN_DATA_MAX_PACKET_SIZE);
	USBD_GS_CAN_PrepareReceive(pdev);
	return USBD_OK;
}

static uint8_t USBD_GS_CAN_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
	usbd_gs_can_purge_to_host_buf(&hGS_CAN);
	is_usb_suspend_cb = false;

	USBD_LL_CloseEP(pdev, GSUSB_ENDPOINT_IN);
	USBD_LL_CloseEP(pdev, GSUSB_ENDPOINT_OUT);
	return USBD_OK;
}

static bool USBD_GS_CAN_IsRequest(const USBD_SetupReqTypedef *req)
{
	/* Composite control callbacks are broadcast. Only handle gs_usb vendor
	 * requests, retaining support for device-recipient requests. */
	return (req->bmRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_VENDOR &&
		((req->bmRequest & 0x1F) == USB_REQ_RECIPIENT_DEVICE ||
		 ((req->bmRequest & 0x1F) == USB_REQ_RECIPIENT_INTERFACE && req->wIndex == 0));
}

static uint8_t USBD_GS_CAN_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
	if (!USBD_GS_CAN_IsRequest(req)) return USBD_OK;

	union ep0 *ep0 = &hGS_CAN.ep0;
	can_data_t *channel = NULL;
	const void *src = NULL;
	size_t len;

	/*
	 * For all "per device" USB control messages
	 * (GS_USB_BREQ_HOST_FORMAT and GS_USB_BREQ_DEVICE_CONFIG) the
	 * Linux gs_usb driver uses a req->wValue = 1.
	 *
	 * All other control messages are "per channel" and specify the
	 * channel number in req->wValue. So check req->wValue for valid
	 * CAN channel.
	 *
	 */
	if (!(req->bRequest == GS_USB_BREQ_HOST_FORMAT || req->bRequest == GS_USB_BREQ_DEVICE_CONFIG)) {
		channel = USBD_GS_CAN_GetChannel(&hGS_CAN, req->wValue);
		if (!channel) {
			goto out_fail;
		}
	}

	if (!IS_ENABLED(CONFIG_CANFD)) {
		switch (req->bRequest) {
			case GS_USB_BREQ_DATA_BITTIMING:
			case GS_USB_BREQ_BT_CONST_EXT:
				goto out_fail;
		}
	}

	if (!IS_ENABLED(CONFIG_TERMINATION)) {
		switch (req->bRequest) {
			case GS_USB_BREQ_SET_TERMINATION:
			case GS_USB_BREQ_GET_TERMINATION:
				goto out_fail;
		}
	}

	switch (req->bRequest) {
		// Host -> Device
		case GS_USB_BREQ_HOST_FORMAT:
			len = sizeof(ep0->data.config);
			break;
		case GS_USB_BREQ_BITTIMING:
			len = sizeof(ep0->data.bittiming);
			break;
		case GS_USB_BREQ_MODE:
			len = sizeof(ep0->data.mode);
			break;
		case GS_USB_BREQ_BT_CONST:
			src = &CAN_btconst;
			len = sizeof(CAN_btconst);
			break;
		case GS_USB_BREQ_DEVICE_CONFIG:
			src = &USBD_GS_CAN_dconf;
			len = sizeof(USBD_GS_CAN_dconf);
			break;
		case GS_USB_BREQ_TIMESTAMP:
			src = &hGS_CAN.sof_timestamp_us;
			len = sizeof(hGS_CAN.sof_timestamp_us);
			break;
		case GS_USB_BREQ_IDENTIFY:
			len = sizeof(ep0->data.identify_mode);
			break;
		case GS_USB_BREQ_DATA_BITTIMING:
			len = sizeof(ep0->data.bittiming);
			break;
		case GS_USB_BREQ_BT_CONST_EXT:
			src = &CAN_btconst_ext;
			len = sizeof(CAN_btconst_ext);
			break;
		case GS_USB_BREQ_SET_TERMINATION:
			len = sizeof(ep0->data.term_state);
			break;
		case GS_USB_BREQ_GET_TERMINATION:
			// TODO: 后续支持
			ep0->data.term_state.state = GS_CAN_TERMINATION_UNSUPPORTED;
			src = &ep0->data.term_state;
			len = sizeof(ep0->data.term_state);
			break;
		case GS_USB_BREQ_GET_STATE:
			can_get_device_state(channel, &ep0->data.state);
			src = &ep0->data.state;
			len = sizeof(ep0->data.state);
			break;
		default:
			goto out_fail;
	}

	if (req->wLength < len) {
		goto out_fail;
	}

	switch (req->bRequest) {
		case GS_USB_BREQ_HOST_FORMAT:
		case GS_USB_BREQ_BITTIMING:
		case GS_USB_BREQ_MODE:
		case GS_USB_BREQ_IDENTIFY:
		case GS_USB_BREQ_DATA_BITTIMING:
		case GS_USB_BREQ_SET_TERMINATION:
			if (req->wLength > sizeof(*ep0)) {
				goto out_fail;
			}

			hGS_CAN.last_setup_request = *req;
			USBD_CtlPrepareRx(pdev, hGS_CAN.ep0.buf, req->wLength);
			break;

		// Device -> Host
		case GS_USB_BREQ_BT_CONST:
		case GS_USB_BREQ_DEVICE_CONFIG:
		case GS_USB_BREQ_TIMESTAMP:
		case GS_USB_BREQ_BT_CONST_EXT:
		case GS_USB_BREQ_GET_TERMINATION:
		case GS_USB_BREQ_GET_STATE:
			USBD_CtlSendData(pdev, (uint8_t *)src, len);
			break;
		default:
			goto out_fail;
	}

	return USBD_OK;

out_fail:
	USBD_CtlError(pdev, req);
	return USBD_FAIL;
}

static uint8_t USBD_GS_CAN_EP0_RxReady(USBD_HandleTypeDef *pdev) {
	/* Use the current transfer, not a saved request from a previous CAN write. */
	if (!USBD_GS_CAN_IsRequest(&pdev->request)) return USBD_OK;

	can_data_t *channel = NULL;
	USBD_SetupReqTypedef *req = &hGS_CAN.last_setup_request;
	const union ep0 *ep0 = &hGS_CAN.ep0;

	/*
	 * The control messages GS_USB_BREQ_HOST_FORMAT and
	 * GS_USB_BREQ_DEVICE_CONFIG are per device, all others are per
	 * channel. The validity of req->wValue has been checked in
	 * USBD_GS_CAN_Config_Request() already.
	 */
	if (!(req->bRequest == GS_USB_BREQ_HOST_FORMAT || req->bRequest == GS_USB_BREQ_DEVICE_CONFIG))
		channel = USBD_GS_CAN_GetChannel(&hGS_CAN, req->wValue);

	if (!IS_ENABLED(CONFIG_CANFD)) {
		switch (req->bRequest) {
			case GS_USB_BREQ_DATA_BITTIMING:
				goto out_fail;
		}
	}

	if (!IS_ENABLED(CONFIG_TERMINATION)) {
		switch (req->bRequest) {
			case GS_USB_BREQ_SET_TERMINATION:
				goto out_fail;
		}
	}

	switch (req->bRequest) {
		case GS_USB_BREQ_HOST_FORMAT:
			/*
			 * The firmware on the original USB2CAN by Geschwister Schneider
			 * Technologie Entwicklungs- und Vertriebs UG exchanges all data
			 * between the host and the device in host byte order. This is done
			 * with the struct gs_host_config::byte_order member, which is sent
			 * first to indicate the desired byte order.
			 *
			 * The widely used open source firmware candleLight doesn't support
			 * this feature and exchanges the data in little endian byte order.
			 */
			break;
		case GS_USB_BREQ_BITTIMING: {
			if (!can_set_bittiming(channel, &ep0->data.bittiming)) goto out_fail;
			break;
		}
		case GS_USB_BREQ_MODE: {
			const struct gs_device_mode *mode = &ep0->data.mode;

			if (mode->mode == GS_CAN_MODE_RESET) {
				can_disable(&hGS_CAN, channel);
			} else if (mode->mode == GS_CAN_MODE_START) {
				if (!can_enable(channel, mode->feature)) goto out_fail;
			}
			break;
		}
		case GS_USB_BREQ_IDENTIFY: {
			if (ep0->data.identify_mode.mode) {
				led.mode = signal_led::Mode::CONTINUES;
			} else {
				led.mode = can_is_enabled(channel) ? signal_led::Mode::NORMAL : signal_led::Mode::OFF;
			}
			break;
		}
		case GS_USB_BREQ_DATA_BITTIMING: {
			if (!can_set_data_bittiming(channel, &ep0->data.bittiming)) goto out_fail;
			break;
		}
		case GS_USB_BREQ_SET_TERMINATION: {
			// TODO: 后续支持
			// set_term(channel, (gs_can_termination_state)ep0->data.term_state.state);
			break;
		}
		default:
			break;
	}

	req->bRequest = 0xFF;
	return USBD_OK;

out_fail:
	return USBD_FAIL;
}

static uint8_t USBD_GS_CAN_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	if (epnum != (GSUSB_ENDPOINT_IN & 0x7F)) return USBD_OK;

    bool was_irq_enabled = disable_irq();
    list_add_tail(&hGS_CAN.to_host_buf->list, &hGS_CAN.list_frame_pool);
    hGS_CAN.to_host_buf = NULL;
    restore_irq(was_irq_enabled);

    return USBD_OK;
}

static uint8_t out_prepare_receive(USBD_HandleTypeDef *pdev) {
	// We can not unconditionally re-enable RX if the pre-allocated
	// buffers are not fully ready.
	for (unsigned i = 0; i < ARRAY_SIZE(hGS_CAN.from_host_buf); ++i) {
		if (!hGS_CAN.from_host_buf[i])
			return USBD_OK;
	}
	USBD_GS_CAN_PrepareReceive(pdev);

	return USBD_OK;
}

// Note that the return value is completely ignored by the stack.
static uint8_t USBD_GS_CAN_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	if (epnum != GSUSB_ENDPOINT_OUT) return USBD_OK;

	can_data_t *channel;

	/* If we do not have a valid incoming frame pointer, something broke. */
	if (!hGS_CAN.from_host_buf[0])
		__BKPT(0);

	uint32_t rxlen = USBD_LL_GetRxDataSize(pdev, epnum);
	if (rxlen < (struct_size(&hGS_CAN.from_host_buf[0]->frame, classic_can, 1))) {
		// Invalid frame length, just ignore it and receive into the same buffer
		// again next time.
		return out_prepare_receive(pdev);
	}

	channel = USBD_GS_CAN_GetChannel(&hGS_CAN, hGS_CAN.from_host_buf[0]->frame.channel);
	if (!channel) {
		return out_prepare_receive(pdev);
	}

	if (IS_ENABLED(CONFIG_CANFD) &&
		hGS_CAN.from_host_buf[0]->frame.flags & GS_CAN_FLAG_FD &&
		rxlen < struct_size(&hGS_CAN.from_host_buf[0]->frame, canfd, 1)) {
		return out_prepare_receive(pdev);
	}

	bool was_irq_enabled = disable_irq();
	// Enqueue the frame we just received.
	list_add_tail(&hGS_CAN.from_host_buf[0]->list, &channel->list_from_host);

	int last;
	for (last = 0; last < (int)ARRAY_SIZE(hGS_CAN.from_host_buf) - 1; ++last)
		hGS_CAN.from_host_buf[last] = hGS_CAN.from_host_buf[last + 1];
	hGS_CAN.from_host_buf[last] = NULL;

	bool rx_ready = USBD_GS_Prefill_RX_Buffers(&hGS_CAN);

	if (rx_ready) {
		restore_irq(was_irq_enabled);

		// All RX buffers are ready. Enable RX.
		USBD_GS_CAN_PrepareReceive(pdev);
	} else {
		restore_irq(was_irq_enabled);

#if defined(USB) || defined(USB_DRD_FS)
		// Not all RX buffers are ready. We got enough in the
		// pre-allocated reserve to ensure callbacks which are going
		// to be called due to USB frames being received into double
		// buffer, but disable further RX. If double buffer not in use,
		// RX would have already been disabled by USB HW.
		USBD_GS_CAN_PrepareReceive(pdev);
		PCD_SET_EP_RX_STATUS(((PCD_HandleTypeDef *)pdev->pData)->Instance,
							 GSUSB_ENDPOINT_OUT,
							 USB_EP_RX_NAK);
#endif
	}
	return USBD_OK;
}

static uint8_t USBD_GS_CAN_SOF(struct _USBD_HandleTypeDef *pdev) {
	hGS_CAN.sof_timestamp_us = timer_get();
	return USBD_OK;
}

USBD_ClassTypeDef USBD_GS_CAN = {
	.Init = USBD_GS_CAN_Init,
	.DeInit = USBD_GS_CAN_DeInit,
	.Setup = USBD_GS_CAN_Setup,
	.EP0_RxReady = USBD_GS_CAN_EP0_RxReady,
	.DataIn = USBD_GS_CAN_DataIn,
	.DataOut = USBD_GS_CAN_DataOut,
	.SOF = USBD_GS_CAN_SOF,
};

void USBD_GS_CAN_ReceiveFromHost(USBD_HandleTypeDef *pdev)
{
	bool was_irq_enabled = disable_irq();

	// Reserve the multiple RX buffers we need to ensure no frame loss
	// with OUT endpoint double-buffering before (re)starting the RX. */
	bool rx_ready = USBD_GS_Prefill_RX_Buffers(&hGS_CAN);

	if (rx_ready)
		USBD_GS_CAN_PrepareReceive(pdev);

	restore_irq(was_irq_enabled);
}

static uint8_t USBD_GS_CAN_Transmit(USBD_HandleTypeDef *pdev, uint8_t *buf, uint16_t len)
{
	if (false == is_usb_suspend_cb) {
		USBD_LL_Transmit(pdev, GSUSB_ENDPOINT_IN, buf, len);
		return USBD_OK;
	} else {
		return USBD_BUSY;
	}
}

static uint8_t USBD_GS_CAN_SendFrame(USBD_HandleTypeDef *pdev,
									 struct gs_host_frame_object *frame_object)
{
	const can_data_t *channel = &hGS_CAN.channels[frame_object->frame.channel];
	const struct gs_host_frame *frame = &frame_object->frame;
	uint8_t buf[CAN_DATA_MAX_PACKET_SIZE];
	uint8_t *send_addr;
	size_t len;

	if (IS_ENABLED(CONFIG_CANFD) &&
		frame->flags & GS_CAN_FLAG_FD) {
		if (channel->feature & GS_CAN_FEATURE_HW_TIMESTAMP) {
			len = struct_size(frame, canfd_ts, 1);
		} else {
			len = struct_size(frame, canfd, 1);
		}
	} else {
		if (channel->feature & GS_CAN_FEATURE_HW_TIMESTAMP) {
			len = struct_size(frame, classic_can_ts, 1);
		} else {
			len = struct_size(frame, classic_can, 1);
		}
	}

	send_addr = (uint8_t *)frame;

	/*
	 * When talking to WinUSB it seems to help a lot if the size of
	 * packet you send equals the max packet size. In this mode, fill
	 * packets out to max packet size and then send.
	 *
	 * Don't know if the above observation is still true for CAN-FD
	 * (CAN-FD frames don't fit into a single the full speed USB
	 * packet of 64 byte), so don't do any padding for CAN-FD frames
	 * for now.
	 */
	if (channel->feature & GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE &&
		!((IS_ENABLED(CONFIG_CANFD) && frame->flags & GS_CAN_FLAG_FD))) {
		memcpy(buf, frame, len);

		// zero rest of buffer
		memset(buf + len, 0, sizeof(buf) - len);
		send_addr = buf;
		len = sizeof(buf);
	}

	return USBD_GS_CAN_Transmit(pdev, send_addr, len);
}

void USBD_GS_CAN_SendToHost(USBD_HandleTypeDef *pdev)
{
	bool was_irq_enabled = disable_irq();
	if (hGS_CAN.to_host_buf) {
		restore_irq(was_irq_enabled);
		return;
	}

	hGS_CAN.to_host_buf = list_first_entry_or_null(&hGS_CAN.list_to_host,
												 struct gs_host_frame_object,
												 list);
	if (!hGS_CAN.to_host_buf) {
		restore_irq(was_irq_enabled);
		return;
	}

	list_del(&hGS_CAN.to_host_buf->list);
	restore_irq(was_irq_enabled);

	uint8_t result = USBD_GS_CAN_SendFrame(pdev, hGS_CAN.to_host_buf);
	if (result == USBD_OK)
		return;

	/*
	 * If USBD_GS_CAN_SendFrame() fails, it will be due to a USB suspend event
	 * (is_usb_suspend_cb == true).
	 * For now the firmware shuts down the CAN interface during suspend and
	 * hcan->to_host_buf is pruged by usbd_gs_can_purge_to_host_buf() during
	 * USBD_GS_CAN_DeInit().
	 */
	was_irq_enabled = disable_irq();
	if (hGS_CAN.to_host_buf) {
		list_add_tail(&hGS_CAN.to_host_buf->list, &hGS_CAN.list_frame_pool);
		hGS_CAN.to_host_buf = NULL;
	}
	restore_irq(was_irq_enabled);
}

// Handle USB suspend event
void USBD_GS_CAN_SuspendCallback(USBD_HandleTypeDef *pdev)
{
	// Disable CAN and go off bus on USB suspend
	for (unsigned int i = 0; i < ARRAY_SIZE(hGS_CAN.channels); i++) {
		can_data_t *channel = &hGS_CAN.channels[i];

		can_disable(&hGS_CAN, channel);
	}

	is_usb_suspend_cb = true;
}

void USBD_GS_CAN_ResumeCallback(USBD_HandleTypeDef __maybe_unused *pdev)
{
	is_usb_suspend_cb = false;
}
