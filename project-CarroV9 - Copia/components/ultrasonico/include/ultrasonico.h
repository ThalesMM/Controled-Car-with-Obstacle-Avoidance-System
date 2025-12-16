// ultrasonico.h

#ifndef ULTRASONICO_H
#define ULTRASONICO_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ultrasonic.h"

// GPIO Pins for Ultrasonic Sensors
#define ULTRASONIC_FRONT_TRIG_GPIO GPIO_NUM_21
#define ULTRASONIC_FRONT_ECHO_GPIO GPIO_NUM_15
#define ULTRASONIC_RIGHT_TRIG_GPIO GPIO_NUM_22
#define ULTRASONIC_RIGHT_ECHO_GPIO GPIO_NUM_23
#define ULTRASONIC_LEFT_TRIG_GPIO GPIO_NUM_18
#define ULTRASONIC_LEFT_ECHO_GPIO GPIO_NUM_5

// Sensor structs - DECLARE as extern
extern ultrasonic_sensor_t front_sensor;
extern ultrasonic_sensor_t right_sensor;
extern ultrasonic_sensor_t left_sensor;

esp_err_t ultrasonic_setup(void);

#endif // ultrasonico_H