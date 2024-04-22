#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "pwm.h"

#define FORWARD_GPIO 27
#define REVERSE_GPIO 17
#define SERVO_GPIO 0

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
    gpio_init(FORWARD_GPIO);
    gpio_set_dir(FORWARD_GPIO, GPIO_IN);
    gpio_pull_up(FORWARD_GPIO);

    gpio_init(REVERSE_GPIO);
    gpio_set_dir(REVERSE_GPIO, GPIO_IN);
    gpio_pull_up(REVERSE_GPIO);

    // main loop
    bool forward = false, reverse = false;

    while (true) {
        forward = !gpio_get(FORWARD_GPIO); // active-low
        reverse = !gpio_get(REVERSE_GPIO);

        if (reverse) {
            pwm_set_chan_level(output_slice_num, output_chan_num, 1000);

            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(50);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(50);
        } else if (forward) {
            pwm_set_chan_level(output_slice_num, output_chan_num, 2000);

            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(100);
        } else {
            pwm_set_chan_level(output_slice_num, output_chan_num, 1500);

            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(100);
        }
    }
}
