/**
 * @file control.h
 * @brief Public interface for the robot's control and sensor tasks.
 *
 * This module manages the main control logic of the robot. It encapsulates
 * the tasks for reading sensor data, processing that data, and controlling
 * the motors. Communication between the sensor task and the control task
 * is handled internally using a FreeRTOS Queue.
 */

#ifndef TASK_CONTROL_H
#define TASK_CONTROL_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <stdbool.h> // For the 'bool' type

/*
 * These includes are not strictly necessary for the function prototypes below,
 * but are often included as a convenience for any file that includes control.h,
 * as these modules are closely related.
 */
#include "motor.h"
#include "ultrasonico.h" // Assuming this is one of your sensor headers
#include "ultrasonic.h"  // Assuming this is the other sensor header
#include "buzzLed.h"

// --- PUBLIC CONSTANTS ---

/**
 * @brief The length of the internal queue used to buffer sensor readings.
 *
 * This defines how many sensor readings can be stored before the sensor task
 * has to wait for the control task to process them.
 */
#define QUEUE_LENGTH 15

/** @brief Maximum distance (in cm) in front before an obstacle is detected. */
#define MAX_DIST_FRONT 25

/** @brief Maximum distance (in cm) to the right for an open path. */
#define MAX_DIST_RIGHT 9

/** @brief Maximum distance (in cm) to the left for an open path. */
#define MAX_DIST_LEFT 9

// Adicione estas linhas no final do arquivo, antes do #endif

extern TaskHandle_t control_task_handle;
extern TaskHandle_t sensor_task_handle;

typedef enum
{
    STOP,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
} joystick_state_t;
/**
 * @brief Define o comando de movimento recebido do controle remoto.
 *
 * Esta função é thread-safe e deve ser chamada pelo módulo de comunicação
 * (ex: ESP-NOW) para atualizar a intenção do piloto.
 *
 * @param command O novo estado do joystick recebido.
 */
void control_set_remote_command(joystick_state_t command);
// --- PUBLIC FUNCTION PROTOTYPES ---

/**
 * @brief Initializes the Inter-Process Communication (IPC) mechanism.
 *
 * This function must be called once before starting any tasks from this module.
 * It sets up the FreeRTOS Queue used for communication between the sensor
 * and control tasks.
 *
 * @return esp_err_t
 *      - ESP_OK on success.
 *      - ESP_FAIL if the queue could not be created.
 */
esp_err_t init_control_ipc(void);

/**
 * @brief Creates and starts the main robot control task.
 *
 * This task is responsible for receiving sensor data and making decisions
 * about the robot's movement. It acts as the "brain" of the robot.
 *
 * @return esp_err_t
 *      - ESP_OK on success.
 *      - ESP_FAIL if the task could not be created.
 */
esp_err_t start_control_task(void);

/**
 * @brief Creates and starts the task for reading ultrasonic sensors.
 *
 * This task continuously measures the distances from the front, right, and
 * left sensors and sends the collected data to the control task via a queue.
 *
 * @return esp_err_t
 *      - ESP_OK on success.
 *      - ESP_FAIL if the task could not be created.
 */
esp_err_t start_sensor_task(void);

/**
 * @brief Creates and starts a simple Right-Left test task.
 *
 * This is a diagnostic task that makes the robot turn left and right in a loop.
 * It can be used to test motor functionality independently of the sensor logic.
 *
 * @return esp_err_t
 *      - ESP_OK on success.
 *      - ESP_FAIL if the task could not be created.
 */
esp_err_t start_RL_task(void);

/**
 * @brief Controls the state of a feedback LED.
 *
 * A utility function to provide visual feedback, e.g., turning on an LED
 * when an obstacle is detected.
 *
 * @param on Set to 'true' to turn the LED on, 'false' to turn it off.
 */
void feedback_buzz_set(bool on);
void feedback_led_setL(bool on);
void feedback_led_setR(bool on);

#endif // TASK_CONTROL_H