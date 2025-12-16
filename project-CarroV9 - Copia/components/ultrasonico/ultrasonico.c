
#include "ultrasonico.h"

// Sensor struct definitions
ultrasonic_sensor_t front_sensor = {
    .trigger_pin = ULTRASONIC_FRONT_TRIG_GPIO,
    .echo_pin = ULTRASONIC_FRONT_ECHO_GPIO};

ultrasonic_sensor_t right_sensor = {
    .trigger_pin = ULTRASONIC_RIGHT_TRIG_GPIO,
    .echo_pin = ULTRASONIC_RIGHT_ECHO_GPIO};

ultrasonic_sensor_t left_sensor = {
    .trigger_pin = ULTRASONIC_LEFT_TRIG_GPIO,
    .echo_pin = ULTRASONIC_LEFT_ECHO_GPIO};

static const char *TAG = "ULTRASONIC";

esp_err_t ultrasonic_setup(void)
{
    esp_err_t ret;

    // Initialize front sensor
    ret = ultrasonic_init(&front_sensor);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Front ultrasonic init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize right sensor
    ret = ultrasonic_init(&right_sensor);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Right ultrasonic init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize left sensor
    ret = ultrasonic_init(&left_sensor);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Left ultrasonic init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "All ultrasonic sensors initialized");
    return ESP_OK;
}
