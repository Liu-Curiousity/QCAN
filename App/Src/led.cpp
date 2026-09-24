#include "led.h"

static signal_led led{
	{LED_RX_GPIO_Port, LED_RX_Pin, true},
	{LED_TX_GPIO_Port, LED_TX_Pin, true}
};

void led_set_mode(led_data_t *leds, led_mode_t mode) {
	led.mode = mode == LED_MODE_OFF ? signal_led::Mode::OFF :
	           mode == LED_MODE_SEQUENCE ? signal_led::Mode::CONTINUES :
	           signal_led::Mode::NORMAL;
}

void led_indicate_trx(led_data_t *leds, led_num_t num) {
	led.blink_request(num == LED_RX ? signal_led::LED_RX : signal_led::LED_TX);
}

void led_update(led_data_t *leds)
{
	led.update();
}
