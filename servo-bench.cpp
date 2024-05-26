#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "pwm.h"

#ifdef DEBUGGING_SCREEN
#include "hardware/i2c.h"
#include "pico-ssd1306/ssd1306.h"
#include "pico-ssd1306/textRenderer/TextRenderer.h"
#include "pico-ssd1306/textRenderer/8x8_font.h"
#endif /* DEBUGGING_SCREEN */

#define ARM_FIRE_GPIO 16
#define DISARM_GPIO 27
#define SWITCH_GPIO 28
#define SERVO_GPIO 0

#ifdef DEBUGGING_SCREEN
i2c_inst * const SCREEN_I2C_PORT = &i2c0_inst;
#define SCREEN_I2C_SCL 21
#define SCREEN_I2C_SDA 20
#endif /* DEBUGGING_SCREEN */

#define DEBOUNCE_TIME_US 5000

class debounce_buffer {
    public:
        debounce_buffer(bool initial_value): stable(initial_value), last(initial_value), last_change_us(0) {}

        bool update(bool current) {
            uint64_t now = time_us_64();

            // if the last value we saw has changed, update the last value
            if (current != this->last) {
                this->last = current;
                this->last_change_us = now;
            }

            // if the last value has been stable for long enough, update the stable value
            if (this->stable != this->last && now - this->last_change_us > DEBOUNCE_TIME_US) {
                this->stable = this->last;
            }

            return this->stable;
        }

    private:
        bool stable;
        bool last;
        uint64_t last_change_us;
};

enum state_t {
    STATE_IDLE = 0,
    STATE_COCKING = 1,
    STATE_COCKED = 2,
    STATE_FIRING = 3,
    STATE_DECOCKING = 4,
    STATE_RESETTING = 5,
    STATE_IDLE_AFTER_RESET = 6,
    STATE_RECOCKING_AFTER_RESET = 7
};

int main(void) {
    stdio_init_all();

    // configure peripherals
    pwm_config cfg;

    // - LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // - servo output
    gpio_init(SERVO_GPIO);
    gpio_set_dir(SERVO_GPIO, GPIO_OUT);
    gpio_set_function(SERVO_GPIO, GPIO_FUNC_PWM);

    cfg = pwm_get_default_config();

    uint output_slice_num = pwm_gpio_to_slice_num(SERVO_GPIO);
    uint output_chan_num = pwm_gpio_to_channel(SERVO_GPIO);

    pwm_init(output_slice_num, &cfg, true);

    // configure the PWM clock divider for a 20ms cycle time and a granularity
    // of 1us/count (1mhz clock divider output)
    //
    // https://en.wikipedia.org/wiki/Servo_control#/media/File:Servomotor_Timing_Diagram.svg
    float clkdiv = ((float)frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS)) / 1000.0f;
    pwm_set_clkdiv(output_slice_num, clkdiv);
    pwm_set_wrap(output_slice_num, 20000);

    // - buttons
    gpio_init(ARM_FIRE_GPIO);
    gpio_set_dir(ARM_FIRE_GPIO, GPIO_IN);
    gpio_pull_up(ARM_FIRE_GPIO);

    gpio_init(DISARM_GPIO);
    gpio_set_dir(DISARM_GPIO, GPIO_IN);
    gpio_pull_up(DISARM_GPIO);

    // - cam position sensing switch
    gpio_init(SWITCH_GPIO);
    gpio_set_dir(SWITCH_GPIO, GPIO_IN);
    gpio_pull_down(SWITCH_GPIO);

#ifdef DEBUGGING_SCREEN
    // - debugging screen
    i2c_init(SCREEN_I2C_PORT, 1000000);
    gpio_set_function(SCREEN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_function(SCREEN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(SCREEN_I2C_SCL);
    gpio_pull_up(SCREEN_I2C_SDA);

    pico_ssd1306::SSD1306 dpy = pico_ssd1306::SSD1306(SCREEN_I2C_PORT, 0x3c, pico_ssd1306::Size::W128xH64);
    pico_ssd1306::drawText(&dpy, font_8x8, "hello world", 0, 0);
    dpy.sendBuffer();
    dpy.clear();
#endif /* DEBUGGING_SCREEN */

    // set initial state
    state_t state;

    if (gpio_get(SWITCH_GPIO)) {
        // gpio being pulled up through closed switch
        state = STATE_COCKED;
    } else {
        // gpio being pulled down by rp2040
        state = STATE_IDLE;
    }

    debounce_buffer db_arm_fire(false);
    debounce_buffer db_disarm(false);
    debounce_buffer db_cam_switch(false);

    while (true) {
        bool arm_fire = db_arm_fire.update(!gpio_get(ARM_FIRE_GPIO)); // active-low
        bool disarm = db_disarm.update(!gpio_get(DISARM_GPIO)); // active-low
        bool cam_switch = db_cam_switch.update(!gpio_get(SWITCH_GPIO));

        // state transitions
        // https://www.figma.com/board/st0NGDv1HwW2Hs8RRE4nCe/flopsy-weapon-state-diagram?node-id=0-1&t=YhJ3ebz0ybV1lXvY-0
        state_t new_state = state;

        switch (state) {
            case STATE_IDLE:
                if (disarm) new_state = STATE_RESETTING;
                else if (arm_fire) new_state = STATE_COCKING;
                break;
            case STATE_COCKING:
                if (disarm) new_state = STATE_RESETTING;
                else if (cam_switch) new_state = STATE_COCKED;
                break;
            case STATE_COCKED:
                if (disarm) new_state = STATE_DECOCKING;
                else if (arm_fire) new_state = STATE_FIRING;
                break;
            case STATE_FIRING:
                if (disarm) new_state = STATE_DECOCKING;
                else if (!cam_switch) new_state = STATE_COCKING;
                break;
            case STATE_DECOCKING:
                if (!cam_switch) new_state = STATE_RESETTING;
                break;
            case STATE_RESETTING:
                if (cam_switch) new_state = STATE_IDLE_AFTER_RESET;
                break;
            case STATE_IDLE_AFTER_RESET:
                if (arm_fire) new_state = STATE_RECOCKING_AFTER_RESET;
                break;
            case STATE_RECOCKING_AFTER_RESET:
                if (disarm) new_state = STATE_IDLE_AFTER_RESET;
                else if (!cam_switch) new_state = STATE_COCKING;
                break;
        }

        if (new_state != state) {
#ifdef DEBUGGING_SCREEN
            char str_out[16];
            snprintf(str_out, 16, "state: %d", new_state);
            pico_ssd1306::drawText(&dpy, font_8x8, str_out, 0, 0);
            snprintf(str_out, 16, "sw: %d", cam_switch);
            pico_ssd1306::drawText(&dpy, font_8x8, str_out, 0, 8);
            dpy.sendBuffer();
            dpy.clear();
#endif /* DEBUGGING_SCREEN */

            state = new_state;
        }

        // servo output
        bool forward = false, reverse = false;

        switch (state) {
            case STATE_COCKING:
            case STATE_FIRING:
            case STATE_RECOCKING_AFTER_RESET:
                forward = true;
                reverse = false;
                break;
            case STATE_DECOCKING:
            case STATE_RESETTING:
                forward = false;
                reverse = true;
                break;
        }

        if (reverse) {
            pwm_set_chan_level(output_slice_num, output_chan_num, 1000);
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
        } else if (forward) {
            pwm_set_chan_level(output_slice_num, output_chan_num, 2000);
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
        } else {
            pwm_set_chan_level(output_slice_num, output_chan_num, 1500);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
        }
    }
}
