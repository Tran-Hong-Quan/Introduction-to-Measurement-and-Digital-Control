#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define MOTOR_PWM_FREQ 1000 // Tần số PWM
#define MOTOR_PWM_TIMER LEDC_TIMER_0
#define MOTOR_PWM_CHANNEL LEDC_CHANNEL_0
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_12_BIT

#define MOTOR_ENA_GPIO 18
#define MOTOR_IN1_GPIO 19
#define MOTOR_IN2_GPIO 21

void motor_control_init()
{
    // Cấu hình GPIO
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << MOTOR_ENA_GPIO) | (1ULL << MOTOR_IN1_GPIO) | (1ULL << MOTOR_IN2_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE};
    gpio_config(&io_conf);

    // Khởi tạo LEDC
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = MOTOR_PWM_TIMER,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .freq_hz = MOTOR_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t pwm_conf = {
        .gpio_num = MOTOR_ENA_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = MOTOR_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = MOTOR_PWM_TIMER,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&pwm_conf);
}

void motor_control(int duty_cycle, int direction)
{
    if (direction == 1)
    {
        gpio_set_level(MOTOR_IN1_GPIO, 1);
        gpio_set_level(MOTOR_IN2_GPIO, 0);
    }
    else
    {
        gpio_set_level(MOTOR_IN1_GPIO, 0);
        gpio_set_level(MOTOR_IN2_GPIO, 1);
    }

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_PWM_CHANNEL, duty_cycle);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_PWM_CHANNEL);
}

void app_main()
{
    motor_control_init();

    while (1)
    {
        // Điều chỉnh động cơ với duty cycle từ 0 đến 4000 (đối với 12-bit PWM)
        for (int duty_cycle = 0; duty_cycle <= 4000; duty_cycle += 500)
        {
            motor_control(duty_cycle, 1); // Điều khiển động cơ theo chiều thuận
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

        // Điều chỉnh động cơ với duty cycle từ 4000 đến 0 (đối với 12-bit PWM)
        for (int duty_cycle = 4000; duty_cycle >= 0; duty_cycle -= 500)
        {
            motor_control(duty_cycle, 0); // Điều khiển động cơ theo chiều ngược lại
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
