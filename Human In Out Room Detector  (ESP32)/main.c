#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_GPIO 2

void app_main() {
    int read_raw;
    adc2_config_channel_atten(ADC2_CHANNEL_3, ADC_ATTEN_11db);
    bool current = 0, next;
    int d = 0;
    gpio_pad_select_gpio(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        esp_err_t r = adc2_get_raw(ADC2_CHANNEL_3, ADC_WIDTH_12Bit, &read_raw);

        if (read_raw < 2000) {
            next = 0;
        } else {
            next = 1;
        }

        if (next == 0 && current == 1) {
            d++;
            printf("So nguoi di vao/ra la: %d\n", d);
        }

        current = next;

        if (next == 0) {
            gpio_set_level(LED_GPIO, 1);
        } else {
            gpio_set_level(LED_GPIO, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}
