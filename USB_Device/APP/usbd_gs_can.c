#include "usbd_gs_can.h"

#include <string.h>
#include <usbd_core.h>

#include "host_frame.h"
#include "can_common.h"
#include "gpio_.h"
#include "timer.h"

volatile bool is_usb_suspend_cb;

static can_data_t *USBD_GS_CAN_GetChannel(USBD_COMPOSITE_HandleTypeDef *hcan,
										  const uint16_t ch)
{
	if (ch < ARRAY_SIZE(hcan->channels)) {
		return &hcan->channels[ch];
	}

	return NULL;
}

static const led_seq_step_t led_identify_seq[] = {
	{ .state = 0x01, .time_in_10ms = 10 },
	{ .state = 0x02, .time_in_10ms = 10 },
	{ .state = 0x00, .time_in_10ms = 0 }
};

// Prefill the buffers from RX from host.
// Return true if pre-allocated buffer list buffer list
// was changed and is now ready, meaning and RX can be
// re-enabled, false otherwise.
// Must be called with IRQ disabled.
static bool USBD_GS_Prefill_RX_Buffers(USBD_COMPOSITE_HandleTypeDef *hcan)
{
	unsigned i;
	bool changed;

	changed = false;
	for (i = 0; i < ARRAY_SIZE(hcan->from_host_buf); ++i) {
		if (hcan->from_host_buf[i])
			continue;

		hcan->from_host_buf[i] = list_first_entry_or_null(&hcan->list_frame_pool,
														  struct gs_host_frame_object,
														  list);
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
	USBD_COMPOSITE_HandleTypeDef *hcan = pdev->pClassData;
	struct gs_host_frame *frame = &hcan->from_host_buf[0]->frame;
	uint16_t size;

	if (IS_ENABLED(CONFIG_CANFD)) {
		size = struct_size(frame, canfd_ts, 1);
	} else {
		size = struct_size(frame, classic_can_ts, 1);
	}

	return USBD_LL_PrepareReceive(pdev, GSUSB_ENDPOINT_OUT, (uint8_t *)frame, size);
}

static void usbd_gs_can_purge_to_host_buf(USBD_COMPOSITE_HandleTypeDef *hcan)
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
	USBD_COMPOSITE_HandleTypeDef *hcan = pdev->pClassData;

	usbd_gs_can_purge_to_host_buf(hcan);
	is_usb_suspend_cb = false;

	USBD_LL_CloseEP(pdev, GSUSB_ENDPOINT_IN);
	USBD_LL_CloseEP(pdev, GSUSB_ENDPOINT_OUT);
	return USBD_OK;
}

static uint8_t USBD_GS_CAN_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
	USBD_COMPOSITE_HandleTypeDef *hcan = pdev->pClassData;
	union ep0 *ep0 = &hcan->ep0;
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
	if (!(req->bRequest == GS_USB_BREQ_HOST_FORMAT ||
		  req->bRequest == GS_USB_BREQ_DEVICE_CONFIG)) {
		channel = USBD_GS_CAN_GetChannel(hcan, req->wValue);
		if (!channel) {
			goto out_fail;
		}
	}

	if (!IS_ENABLED(CONFIG_CANFD)) {
		switch (req->bRequest) {
			case GS_USB_BREQ_DATA_BITTIMING:
			case GS_USB_BREQ_BT_CONST_EXT:
			case GS_USB_BREQ_GET_TDC_CONST:
			case GS_USB_BREQ_SET_TDC:
			case GS_USB_BREQ_GET_TDC:
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

	if (!IS_ENABLED(CONFIG_CAN_FILTER)) {
		switch (req->bRequest) {
			case GS_USB_BREQ_SET_FILTER:
			case GS_USB_BREQ_GET_FILTER:
				goto out_fail;
		}
	}

	switch (req->bRequest) {
		// Host -> Device
		case GS_USB_BREQ_HOST_FORMAT:
			len = sizeof(ep0->config);
			break;
		case GS_USB_BREQ_BITTIMING:
			len = sizeof(ep0->bittiming);
			break;
		case GS_USB_BREQ_MODE:
			len = sizeof(ep0->mode);
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
			src = &hcan->sof_timestamp_us;
			len = sizeof(hcan->sof_timestamp_us);
			break;
		case GS_USB_BREQ_IDENTIFY:
			len = sizeof(ep0->identify_mode);
			break;
		case GS_USB_BREQ_DATA_BITTIMING:
			len = sizeof(ep0->bittiming);
			break;
		case GS_USB_BREQ_BT_CONST_EXT:
			src = &CAN_btconst_ext;
			len = sizeof(CAN_btconst_ext);
			break;
		case GS_USB_BREQ_SET_TERMINATION:
			len = sizeof(ep0->term_state);
			break;
		case GS_USB_BREQ_GET_TERMINATION:
			ep0->term_state.state = get_term(channel);
			src = &ep0->term_state;
			len = sizeof(ep0->term_state);
			break;
		case GS_USB_BREQ_GET_STATE:
			can_get_device_state(channel, &ep0->state);
			src = &ep0->state;
			len = sizeof(ep0->state);
			break;
		case GS_USB_BREQ_SET_FILTER:
			len = sizeof(ep0->filter);
			break;
		case GS_USB_BREQ_GET_FILTER:
			src = &CAN_filter_info;
			len = sizeof(CAN_filter_info);
			break;
		case GS_USB_BREQ_GET_TDC_CONST:
			src = &CAN_tdc_const;
			len = sizeof(CAN_tdc_const);
			break;
		case GS_USB_BREQ_SET_TDC:
			len = sizeof(ep0->tdc);
			break;
		case GS_USB_BREQ_GET_TDC:
			can_get_device_tdc(channel, &ep0->tdc);
			src = &ep0->tdc;
			len = sizeof(ep0->tdc);
			break;
		case GS_USB_BREQ_BUS_OFF_RECOVERY:
			len = 0;
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
		case GS_USB_BREQ_SET_FILTER:
		case GS_USB_BREQ_SET_TDC:
		case GS_USB_BREQ_BUS_OFF_RECOVERY:
			if (req->wLength > sizeof(*ep0)) {
				goto out_fail;
			}

			hcan->last_setup_request = *req;
			USBD_CtlPrepareRx(pdev, hcan->ep0.buf, req->wLength);
			break;

		// Device -> Host
		case GS_USB_BREQ_BT_CONST:
		case GS_USB_BREQ_DEVICE_CONFIG:
		case GS_USB_BREQ_TIMESTAMP:
		case GS_USB_BREQ_BT_CONST_EXT:
		case GS_USB_BREQ_GET_TERMINATION:
		case GS_USB_BREQ_GET_STATE:
		case GS_USB_BREQ_GET_FILTER:
		case GS_USB_BREQ_GET_TDC_CONST:
		case GS_USB_BREQ_GET_TDC:
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
	USBD_COMPOSITE_HandleTypeDef *hcan = pdev->pClassData;
	can_data_t *channel = NULL;
	USBD_SetupReqTypedef *req = &hcan->last_setup_request;
	const union ep0 *ep0 = &hcan->ep0;

	/*
	 * The control messages GS_USB_BREQ_HOST_FORMAT and
	 * GS_USB_BREQ_DEVICE_CONFIG are per device, all others are per
	 * channel. The validity of req->wValue has been checked in
	 * USBD_GS_CAN_Config_Request() already.
	 */
	if (!(req->bRequest == GS_USB_BREQ_HOST_FORMAT ||
		  req->bRequest == GS_USB_BREQ_DEVICE_CONFIG))
		channel = USBD_GS_CAN_GetChannel(hcan, req->wValue);

	if (!IS_ENABLED(CONFIG_CANFD)) {
		switch (req->bRequest) {
			case GS_USB_BREQ_DATA_BITTIMING:
			case GS_USB_BREQ_SET_TDC:
				goto out_fail;
		}
	}

	if (!IS_ENABLED(CONFIG_TERMINATION)) {
		switch (req->bRequest) {
			case GS_USB_BREQ_SET_TERMINATION:
				goto out_fail;
		}
	}

	if (!IS_ENABLED(CONFIG_CAN_FILTER)) {
		switch (req->bRequest) {
			case GS_USB_BREQ_SET_FILTER:
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
			const struct gs_device_bittiming *timing = &ep0->bittiming;

			if (!can_check_bittiming_ok(&CAN_btconst.btc, timing))
				goto out_fail;

			can_set_bittiming(channel, timing);
			break;
		}
		case GS_USB_BREQ_MODE: {
			const struct gs_device_mode *mode = &ep0->mode;

			if (mode->mode == GS_CAN_MODE_RESET) {
				can_disable(hcan, channel);
			} else if (mode->mode == GS_CAN_MODE_START) {
				if (!can_check_feature_ok(channel, mode->feature))
					goto out_fail;

				can_enable(channel, mode->feature);
			}
			break;
		}
		case GS_USB_BREQ_IDENTIFY: {
			const struct gs_identify_mode *imode = &ep0->identify_mode;

			if (imode->mode) {
				led_run_sequence(&channel->leds, led_identify_seq, -1);
			} else {
				led_set_mode(&channel->leds, can_is_enabled(channel) ?
							 LED_MODE_NORMAL : LED_MODE_OFF);
			}
			break;
		}
		case GS_USB_BREQ_DATA_BITTIMING: {
			const struct gs_device_bittiming *timing = &ep0->bittiming;

			if (!can_check_bittiming_ok(&CAN_btconst_ext.dbtc, timing))
				goto out_fail;

			can_set_data_bittiming(channel, timing);
			break;
		}
		case GS_USB_BREQ_SET_TERMINATION: {
			const struct gs_device_termination_state *term_state = &ep0->term_state;

			set_term(channel, term_state->state);
			break;
		}
		case GS_USB_BREQ_SET_FILTER: {
			const struct gs_device_filter *filter = &ep0->filter;

			if (!can_check_filter_ok(filter) || can_is_enabled(channel))
				goto out_fail;

			can_set_filter(channel, filter);
			break;
		}
		case GS_USB_BREQ_SET_TDC: {
			const struct gs_device_tdc *tdc = &ep0->tdc;

			if (can_is_enabled(channel) || !can_check_tdc_ok(&CAN_tdc_const, tdc))
				goto out_fail;

			can_set_tdc(channel, tdc);
			break;
		}
		case GS_USB_BREQ_BUS_OFF_RECOVERY:
			if (!can_is_enabled(channel) || !can_check_bus_off_recovery_ok(channel))
				goto out_fail;

			can_schedule_bus_off_recovery(channel, 0);
			break;

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
    USBD_COMPOSITE_HandleTypeDef *hcan = pdev->pClassData;

    bool was_irq_enabled = disable_irq();
    list_add_tail(&hcan->to_host_buf->list, &hcan->list_frame_pool);
    hcan->to_host_buf = NULL;
    restore_irq(was_irq_enabled);

    return USBD_OK;
}

// Note that the return value is completely ignored by the stack.
static uint8_t USBD_GS_CAN_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	USBD_COMPOSITE_HandleTypeDef *hcan = pdev->pClassData;
	can_data_t *channel;

	/* If we do not have a valid incoming frame pointer, something broke. */
	assert_basic(hcan->from_host_buf[0]);

	uint32_t rxlen = USBD_LL_GetRxDataSize(pdev, epnum);
	if (rxlen < (struct_size(&hcan->from_host_buf[0]->frame, classic_can, 1))) {
		// Invalid frame length, just ignore it and receive into the same buffer
		// again next time.
		goto out_prepare_receive;
	}

	channel = USBD_GS_CAN_GetChannel(hcan, hcan->from_host_buf[0]->frame.channel);
	if (!channel) {
		goto out_prepare_receive;
	}

	if (IS_ENABLED(CONFIG_CANFD) &&
		hcan->from_host_buf[0]->frame.flags & GS_CAN_FLAG_FD &&
		rxlen < struct_size(&hcan->from_host_buf[0]->frame, canfd, 1)) {
		goto out_prepare_receive;
	}

	bool was_irq_enabled = disable_irq();
	// Enqueue the frame we just received.
	list_add_tail(&hcan->from_host_buf[0]->list, &channel->list_from_host);

	int last;
	for (last = 0; last < (int)ARRAY_SIZE(hcan->from_host_buf) - 1; ++last)
		hcan->from_host_buf[last] = hcan->from_host_buf[last + 1];
	hcan->from_host_buf[last] = NULL;

	bool rx_ready = USBD_GS_Prefill_RX_Buffers(hcan);

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

out_prepare_receive:
	// We can not unconditionally re-enable RX if the pre-allocated
	// buffers are not fully ready.
	for (unsigned i = 0; i < ARRAY_SIZE(hcan->from_host_buf); ++i) {
		if (!hcan->from_host_buf[i])
			return USBD_OK;
	}
	USBD_GS_CAN_PrepareReceive(pdev);

	return USBD_OK;
}

static uint8_t USBD_GS_CAN_SOF(struct _USBD_HandleTypeDef *pdev) {
	USBD_COMPOSITE_HandleTypeDef *hcan = pdev->pClassData;
	hcan->sof_timestamp_us = timer_get();
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

uint8_t USBD_GS_CAN_Init_(USBD_COMPOSITE_HandleTypeDef *hcan, USBD_HandleTypeDef *pdev)
{
	pdev->pClassData = hcan;

	return USBD_OK;
}

void USBD_GS_CAN_ReceiveFromHost(USBD_HandleTypeDef *pdev)
{
	USBD_COMPOSITE_HandleTypeDef *hcan = pdev->pClassData;

	bool was_irq_enabled = disable_irq();

	// Reserve the multiple RX buffers we need to ensure no frame loss
	// with OUT endpoint double-buffering before (re)starting the RX. */
	bool rx_ready = USBD_GS_Prefill_RX_Buffers(hcan);

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
	USBD_COMPOSITE_HandleTypeDef *hcan = pdev->pClassData;
	const can_data_t *channel = gs_host_frame_object_get_channel(hcan, frame_object);
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
	USBD_COMPOSITE_HandleTypeDef *hcan = pdev->pClassData;

	bool was_irq_enabled = disable_irq();
	if (hcan->to_host_buf) {
		restore_irq(was_irq_enabled);
		return;
	}

	hcan->to_host_buf = list_first_entry_or_null(&hcan->list_to_host,
												 struct gs_host_frame_object,
												 list);
	if (!hcan->to_host_buf) {
		restore_irq(was_irq_enabled);
		return;
	}

	list_del(&hcan->to_host_buf->list);
	restore_irq(was_irq_enabled);

	uint8_t result = USBD_GS_CAN_SendFrame(pdev, hcan->to_host_buf);
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
	if (hcan->to_host_buf) {
		list_add_tail(&hcan->to_host_buf->list, &hcan->list_frame_pool);
		hcan->to_host_buf = NULL;
	}
	restore_irq(was_irq_enabled);
}

bool USBD_GS_CAN_DfuDetachRequested(USBD_HandleTypeDef *pdev)
{
	USBD_COMPOSITE_HandleTypeDef *hcan = pdev->pClassData;

	return hcan->dfu_detach_requested;
}

// Handle USB suspend event
void USBD_GS_CAN_SuspendCallback(USBD_HandleTypeDef *pdev)
{
	// Disable CAN and go off bus on USB suspend
	USBD_COMPOSITE_HandleTypeDef *hcan = pdev->pClassData;

	for (unsigned int i = 0; i < ARRAY_SIZE(hcan->channels); i++) {
		can_data_t *channel = &hcan->channels[i];

		can_disable(hcan, channel);
	}

	is_usb_suspend_cb = true;
}

void USBD_GS_CAN_ResumeCallback(USBD_HandleTypeDef __maybe_unused *pdev)
{
	is_usb_suspend_cb = false;
}

void usbd_gs_can_purge_from_host_list_by_channel(USBD_COMPOSITE_HandleTypeDef *hcan,
												 struct can_channel *channel)
{
	list_splice_tail_init_locked(&channel->list_from_host, &hcan->list_frame_pool);
}

void usbd_gs_can_purge_to_host_list_by_channel(USBD_COMPOSITE_HandleTypeDef *hcan,
											   const struct can_channel *channel)
{
	/*
	 * If we only have one channel, all object of hcan->list_to_host belong to this one channel.
	 * Move the complete list to the frame pool.
	 */
	if (NUM_CAN_CHANNEL == 1) {
		list_splice_tail_init_locked(&hcan->list_to_host, &hcan->list_frame_pool);

		return;
	}

	/*
	 * For more than one channel, iterate over each object in hcan->list_to_host.
	 */
	struct gs_host_frame_object *iter, *next;
	const uint8_t channel_nr = can_channel_get_nr(channel);
	const bool was_irq_enabled = disable_irq();

	list_for_each_entry_safe(iter, next, &hcan->list_to_host, list) {
		if (gs_host_frame_object_get_channel_nr(iter) == channel_nr) {
			list_move_tail(&iter->list, &hcan->list_frame_pool);
		}
	}

	restore_irq(was_irq_enabled);
}
