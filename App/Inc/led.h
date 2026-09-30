#pragma once

#include <gpio.h>

class led {
public:
    led(const GPIO_TypeDef *port, const uint16_t pin, const bool is_active_high) :
        port(port), pin(pin), is_active_high(is_active_high) {}

    void set(bool state) const {
        if (is_active_high) state = !state;
        HAL_GPIO_WritePin(const_cast<GPIO_TypeDef *>(port), pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

private:
    const GPIO_TypeDef *port = nullptr;
    const uint16_t pin = 0;
    const bool is_active_high;
};

class signal_led {
    class blink_led : public led {
    public:
        using led::led;
        blink_led(const led& source) : led(source) {}

        void blink(const uint32_t now, const uint32_t on_time, const uint32_t off_time) {
            if (blink_request) {
                blink_request = false;
                if (now > on_until && now > off_until) {
                    off_until = now + off_time;
                    on_until = off_until + on_time;
                }
            }
            this->set(now > off_until);
        }

        bool blink_request{false};

    private:
        uint32_t on_until{0};
        uint32_t off_until{0};
    };

public:
    constexpr static uint32_t NORMAL_ON_TIME = 15;      // led blink on time in ms
    constexpr static uint16_t NORMAL_OFF_TIME = 30;     // led blink off time in ms
    constexpr static uint32_t ERROR_ON_TIME = 100;      // led blink on time in ms
    constexpr static uint16_t ERROR_OFF_TIME = 100;     // led blink off time in ms
    constexpr static uint32_t CONTINUES_ON_TIME = 100;  // led blink on time in ms
    constexpr static uint16_t CONTINUES_OFF_TIME = 100; // led blink off time in ms

    enum led_num {
        LED_RX = 0,
        LED_TX = 1,
        LED_MAX
    };

    enum class Mode {
        OFF,
        NORMAL,
        ERROR,
        CONTINUES
    };

    signal_led(const led led_rx, const led led_tx) : leds{[LED_RX] = led_rx, [LED_TX] = led_tx} {}

    Mode mode{Mode::OFF};

    void blink_request(const led_num num) {
        leds[num].blink_request = true;
    }

    void update() {
        // const uint32_t now = HAL_GetTick();

        static uint32_t next_update = 0;
        const uint32_t now = HAL_GetTick();
        if (now < next_update) return;
        next_update = now + 10;

        switch (mode) {
            case Mode::OFF:
                leds[LED_RX].set(false);
                leds[LED_TX].set(false);
                break;

            case Mode::NORMAL:
                leds[LED_RX].blink(now, NORMAL_ON_TIME, NORMAL_OFF_TIME);
                leds[LED_TX].blink(now, NORMAL_ON_TIME, NORMAL_OFF_TIME);
                break;
            case Mode::ERROR:
                leds[LED_RX].blink(now, ERROR_ON_TIME, ERROR_OFF_TIME);
                blink_request(LED_RX);
                leds[LED_TX].set(false);
                break;
            case Mode::CONTINUES:
                leds[LED_RX].blink(now, CONTINUES_ON_TIME, CONTINUES_OFF_TIME);
                leds[LED_TX].blink(now, CONTINUES_ON_TIME, CONTINUES_OFF_TIME);
                blink_request(LED_RX);
                blink_request(LED_TX);
                break;

            default:
                break;
        }
    }

private:
    blink_led leds[LED_MAX];
};