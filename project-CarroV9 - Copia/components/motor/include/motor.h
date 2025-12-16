#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

// Defines for Motor A (Left motor)
#define MOTOR_A_IN1_GPIO GPIO_NUM_26 // Left motor direction 1
#define MOTOR_A_IN2_GPIO GPIO_NUM_27 // Left motor direction 2
#define MOTOR_A_ENA_GPIO GPIO_NUM_14 // Left motor PWM
#define MOTOR_A_LEDC_CHANNEL LEDC_CHANNEL_0

// Defines for Motor B (Right motor)
#define MOTOR_B_IN3_GPIO GPIO_NUM_25 // Right motor direction 1
#define MOTOR_B_IN4_GPIO GPIO_NUM_33 // Right motor direction 2
#define MOTOR_B_ENB_GPIO GPIO_NUM_12 // Right motor PWM
#define MOTOR_B_LEDC_CHANNEL LEDC_CHANNEL_1

// LEDC configuration
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY (5000)

// Duty cycle for motor speed
#define DUTY_FORWARD_A 900      // Base duty cycle (out of 1023 for 10-bit resolution)
#define MOTOR_A_DUTY_OFFSET 20 // Offset for Motor A to balance speed
#define DUTY_FORWARD_B DUTY_FORWARD_A - MOTOR_A_DUTY_OFFSET // Base duty cycle (out of 1023 for 10-bit resolution)

// Function prototypes
esp_err_t setup_hardware(void);
void move_forward(void);
void move_backward(void);
void move_left(void);
void move_right(void);
void move_stop(void);

#endif // MOTOR_CONTROL_H