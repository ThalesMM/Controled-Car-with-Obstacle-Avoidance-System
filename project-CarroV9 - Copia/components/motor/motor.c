#include "motor.h"


static const char *TAG = "MOTOR_CONTROL";

esp_err_t setup_hardware(void)
{
    esp_err_t ret;

    // Configure Motor A GPIOs
    ret = gpio_set_direction(MOTOR_A_IN1_GPIO, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set Motor A IN1 direction: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = gpio_set_direction(MOTOR_A_IN2_GPIO, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set Motor A IN2 direction: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(MOTOR_A_IN1_GPIO, 0);
    gpio_set_level(MOTOR_A_IN2_GPIO, 0);

    // Configure Motor B GPIOs
    ret = gpio_set_direction(MOTOR_B_IN3_GPIO, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set Motor B IN3 direction: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = gpio_set_direction(MOTOR_B_IN4_GPIO, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set Motor B IN4 direction: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(MOTOR_B_IN3_GPIO, 0);
    gpio_set_level(MOTOR_B_IN4_GPIO, 0);

    // LEDC timer for PWM
    ledc_timer_config_t motor_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ret = ledc_timer_config(&motor_timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    // LEDC channel for Motor A
    ledc_channel_config_t ch_a = {
        .gpio_num = MOTOR_A_ENA_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = MOTOR_A_LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0};
    ret = ledc_channel_config(&ch_a);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure LEDC channel A: %s", esp_err_to_name(ret));
        return ret;
    }

    // LEDC channel for Motor B
    ledc_channel_config_t ch_b = {
        .gpio_num = MOTOR_B_ENB_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = MOTOR_B_LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0};
    ret = ledc_channel_config(&ch_b);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure LEDC channel B: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Hardware initialized (Motors A and B).");
    return ESP_OK;
}

void move_forward(void)
{
    // Motor B forward: IN3 low, IN4 high
    
    // Motor A forward: IN1 low, IN2 high
    gpio_set_level(MOTOR_A_IN1_GPIO, 0);
    gpio_set_level(MOTOR_A_IN2_GPIO, 1);
    ledc_set_duty(LEDC_MODE, MOTOR_A_LEDC_CHANNEL, DUTY_FORWARD_A);
    ledc_update_duty(LEDC_MODE, MOTOR_A_LEDC_CHANNEL);
    gpio_set_level(MOTOR_B_IN3_GPIO, 0);
    gpio_set_level(MOTOR_B_IN4_GPIO, 1);
    ledc_set_duty(LEDC_MODE, MOTOR_B_LEDC_CHANNEL, DUTY_FORWARD_B);
    //ESP_LOGI(TAG, "Motor A forward: IN1=0, IN2=1");
    //ESP_LOGI(TAG, "Motor B forward: IN3=0, IN4=1");
    ledc_update_duty(LEDC_MODE, MOTOR_B_LEDC_CHANNEL);

}

void move_forward2(void)
{
    

    vTaskDelay(pdMS_TO_TICKS(50));

    // Motor A forward: IN1 low, IN2 high
    gpio_set_level(MOTOR_A_IN1_GPIO, 0);
    gpio_set_level(MOTOR_A_IN2_GPIO, 1);
    ledc_set_duty(LEDC_MODE, MOTOR_A_LEDC_CHANNEL, DUTY_FORWARD_A);
    ledc_update_duty(LEDC_MODE, MOTOR_A_LEDC_CHANNEL);
    //ESP_LOGI(TAG, "Motor A forward: IN1=0, IN2=1");

    vTaskDelay(pdMS_TO_TICKS(50));

    // Motor B forward: IN3 low, IN4 high

    gpio_set_level(MOTOR_B_IN3_GPIO, 0);
    gpio_set_level(MOTOR_B_IN4_GPIO, 1);
    ledc_set_duty(LEDC_MODE, MOTOR_B_LEDC_CHANNEL, DUTY_FORWARD_B);
    ledc_update_duty(LEDC_MODE, MOTOR_B_LEDC_CHANNEL);
    //ESP_LOGI(TAG, "Motor B forward: IN3=0, IN4=1");
}

void move_backward(void)
{
    // Motor A backward: IN1 high, IN2 low
    gpio_set_level(MOTOR_A_IN1_GPIO, 1);
    gpio_set_level(MOTOR_A_IN2_GPIO, 0);
    ledc_set_duty(LEDC_MODE, MOTOR_A_LEDC_CHANNEL, DUTY_FORWARD_A);
    ledc_update_duty(LEDC_MODE, MOTOR_A_LEDC_CHANNEL);
    //ESP_LOGI(TAG, "Motor A backward: IN1=1, IN2=0");

    // Motor B backward: IN3 high, IN4 low
    gpio_set_level(MOTOR_B_IN3_GPIO, 1);
    gpio_set_level(MOTOR_B_IN4_GPIO, 0);
    ledc_set_duty(LEDC_MODE, MOTOR_B_LEDC_CHANNEL, DUTY_FORWARD_B);
    ledc_update_duty(LEDC_MODE, MOTOR_B_LEDC_CHANNEL);
    //ESP_LOGI(TAG, "Motor B backward: IN3=1, IN4=0");
}

void move_left(void)
{
    // Stop Motor A

    gpio_set_level(MOTOR_A_IN1_GPIO, 1);
    gpio_set_level(MOTOR_A_IN2_GPIO, 0);
    ledc_set_duty(LEDC_MODE, MOTOR_A_LEDC_CHANNEL, DUTY_FORWARD_A);
    ledc_update_duty(LEDC_MODE, MOTOR_A_LEDC_CHANNEL);
    //ESP_LOGI(TAG, "IN1=1, IN2=0");


    // Motor B forward: IN3 low, IN4 high
    gpio_set_level(MOTOR_B_IN3_GPIO, 0);
    gpio_set_level(MOTOR_B_IN4_GPIO, 1);
    ledc_set_duty(LEDC_MODE, MOTOR_B_LEDC_CHANNEL, DUTY_FORWARD_B);
    ledc_update_duty(LEDC_MODE, MOTOR_B_LEDC_CHANNEL);
    //ESP_LOGI(TAG, "IN3=0, IN4=1");
}

void move_right(void)
{
    // Motor A forward: IN1 low, IN2 high
    gpio_set_level(MOTOR_A_IN1_GPIO, 0);
    gpio_set_level(MOTOR_A_IN2_GPIO, 1);
    ledc_set_duty(LEDC_MODE, MOTOR_A_LEDC_CHANNEL, DUTY_FORWARD_A);
    ledc_update_duty(LEDC_MODE, MOTOR_A_LEDC_CHANNEL);
    //ESP_LOGI(TAG, "IN1=0, IN2=1");

    // Motor B backward: IN3 high, IN4 low
    gpio_set_level(MOTOR_B_IN3_GPIO, 1);
    gpio_set_level(MOTOR_B_IN4_GPIO, 0);
    ledc_set_duty(LEDC_MODE, MOTOR_B_LEDC_CHANNEL, DUTY_FORWARD_B);
    ledc_update_duty(LEDC_MODE, MOTOR_B_LEDC_CHANNEL);
    //ESP_LOGI(TAG, "IN3=1, IN4=0");
}

void move_stop(void)
{
    // Stop both motors
    ledc_set_duty(LEDC_MODE, MOTOR_A_LEDC_CHANNEL, 0);
    ledc_set_duty(LEDC_MODE, MOTOR_B_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, MOTOR_A_LEDC_CHANNEL);
    ledc_update_duty(LEDC_MODE, MOTOR_B_LEDC_CHANNEL);
    gpio_set_level(MOTOR_A_IN1_GPIO, 0);
    gpio_set_level(MOTOR_A_IN2_GPIO, 0);
    gpio_set_level(MOTOR_B_IN3_GPIO, 0);
    gpio_set_level(MOTOR_B_IN4_GPIO, 0);
    ESP_LOGI(TAG, "Motors A and B stopped");
}