/**
 * @file control.c
 * @brief Implementation of the robot's control and sensor tasks.
 *
 * This file contains the core logic for the robot's autonomous navigation.
 * It uses a producer-consumer model with two main tasks:
 * 1. sensor_task: Acts as the producer, reading distance values from three
 *    ultrasonic sensors and sending them as a single data packet.
 * 2. control_task: Acts as the consumer, receiving the sensor data and
 *    executing movement commands based on a simple avoidance algorithm.
 *
 * Communication between these tasks is managed by a FreeRTOS Queue, which
 * ensures thread-safe and synchronized data transfer.
 */

#include <stdio.h>
#include "control.h"
#include <esp_log.h>

// --- PRIVATE DEFINITIONS ---

// TAG for logging
static const char *TAG = "TASK_CONTROL";

/**
 * @brief A structure to hold a complete, synchronized set of sensor readings.
 *
 * Using a struct ensures that the control task always works with distance
 * measurements that were taken at the same point in time, avoiding data inconsistency.
 */
typedef struct
{
    float front_distance;
    float right_distance;
    float left_distance;
} sensor_data_t;

// --- SHARED RESOURCES ---

/**
 * @brief The FreeRTOS queue for passing sensor data from the producer to the consumer.
 *
 * This single queue replaces the complex system of multiple buffers and semaphores,
 * simplifying the code and reducing the risk of concurrency bugs.
 */
static QueueHandle_t sensor_data_queue;

TaskHandle_t control_task_handle = NULL;
TaskHandle_t sensor_task_handle = NULL;

static volatile joystick_state_t g_remote_command = STOP;

/**
 * @brief Implementação da função pública para atualizar o comando remoto.
 */
void control_set_remote_command(joystick_state_t command)
{
    g_remote_command = command;
}

// --- TASK IMPLEMENTATIONS ---

/**
 * @brief The main control task (Consumer).
 *
 * Waits for sensor data to arrive in the queue, then decides which direction
 * to move the robot. If no data arrives within a timeout, it stops the motors
 * for safety.
 */
static void control_task(void *pvParameters)
{
    sensor_data_t received_data;

    // --- SUBSTITUA O CONTEÚDO DO SEU while(1) POR ESTE BLOCO ---
    while (1)
    {
        // 1. Lê a intenção atual do piloto a partir da variável global.
        joystick_state_t current_command = g_remote_command;

        // 2. Aguarda por dados frescos dos sensores.
        if (xQueueReceive(sensor_data_queue, &received_data, pdMS_TO_TICKS(200)) == pdTRUE)
        {
            // Log dos dados recebidos para depuração
            ESP_LOGI(TAG, "Comando: %d | Sensores: L=%.1f, F=%.1f, R=%.1f",
                     current_command, received_data.left_distance, received_data.front_distance, received_data.right_distance);

            // 3. Lógica de decisão principal (Árbitro)
            switch (current_command)
            {
            case FORWARD:
                // Se o piloto quer ir para FRENTE, ativamos a lógica de desvio de obstáculos.
                // (Esta é a sua lógica original, reutilizada aqui)
                if (received_data.front_distance > MAX_DIST_FRONT && received_data.right_distance > MAX_DIST_RIGHT && received_data.left_distance > MAX_DIST_LEFT)
                {
                    feedback_buzz_set(false);
                    feedback_ledR_set(false);
                    feedback_ledL_set(false);
                    ESP_LOGI(TAG, "Forward...");
                    move_forward();
                }
                else if (received_data.right_distance > received_data.left_distance && received_data.right_distance > MAX_DIST_RIGHT && received_data.left_distance > MAX_DIST_LEFT)
                {
                    feedback_buzz_set(1);
                    feedback_ledR_set(1);
                    feedback_ledL_set(false);
                    ESP_LOGI(TAG, "Right...");
                    move_right();
                }
                else if (received_data.left_distance > received_data.right_distance && received_data.right_distance > MAX_DIST_RIGHT && received_data.left_distance > MAX_DIST_LEFT)
                {
                    feedback_buzz_set(1);
                    feedback_ledR_set(0);
                    feedback_ledL_set(1);
                    ESP_LOGI(TAG, "Left...");
                    move_left();
                }
                else
                {
                    feedback_buzz_set(1);
                    feedback_ledR_set(1);
                    feedback_ledL_set(1);
                    ESP_LOGI(TAG, "Backward...");
                    move_backward();
                }
                break;

            case BACKWARD:
                // Se o piloto quer ir para TRÁS, obedecemos diretamente.
                feedback_ledR_set(1);
                feedback_ledL_set(1);
                ESP_LOGI(TAG, "Backward...");
                move_backward();
                break;

            case LEFT:
                // Se o piloto quer virar para a ESQUERDA, verificamos se o caminho está livre.
                if ( received_data.right_distance > MAX_DIST_RIGHT)
                {
                    feedback_buzz_set(0);
                    feedback_ledR_set(1);
                    feedback_ledL_set(0);
                    ESP_LOGI(TAG, "Left...");
                    move_left();
                }
                else
                {
                    feedback_buzz_set(1);
                    feedback_ledR_set(1);
                    feedback_ledL_set(1);
                    ESP_LOGI(TAG, "Backward...");
                    move_backward();
                }
                break;

            case RIGHT:
                // Se o piloto quer virar para a DIREITA, verificamos se o caminho está livre.
                if ( received_data.left_distance > MAX_DIST_LEFT)
                {
                    feedback_buzz_set(0);
                    feedback_ledR_set(0);
                    feedback_ledL_set(1);
                    ESP_LOGI(TAG, "Right...");
                    move_right();
                }
                else
                {
                    feedback_buzz_set(1);
                    feedback_ledR_set(1);
                    feedback_ledL_set(1);
                    ESP_LOGI(TAG, "Backward...");
                    move_backward();
                }
                break;

            case STOP:
                move_stop();
                feedback_buzz_set(0);
                feedback_ledR_set(0);
                feedback_ledL_set(0);
                ESP_LOGI(TAG, "Stop...");
                break;
            default:
                // Se o piloto soltou o joystick, o carrinho PARA.
                move_stop();
                feedback_buzz_set(0);
                feedback_ledR_set(0);
                feedback_ledL_set(0);
                ESP_LOGI(TAG, "Stop...");
                break;
            }
        }
        else
        {
            // Se não recebermos dados dos sensores (timeout), paramos por segurança.
            ESP_LOGE(TAG, "Timeout nos sensores!");
            move_stop();
        }
    }
    // --- FIM DO BLOCO DE SUBSTITUIÇÃO ---
}

/**
 * @brief The sensor reading task (Producer).
 *
 * Periodically measures the distance from all three ultrasonic sensors,
 * packages the data into a sensor_data_t struct, and sends it to the control_task
 * via the queue.
 */
static void sensor_task(void *pvParameters)
{
    sensor_data_t current_measurements; // Struct to hold the latest readings

    while (1)
    {
        esp_err_t ret_front, ret_right, ret_left;

        // Measure distances from all sensors
        ret_front = ultrasonic_measure(&front_sensor, 400, &current_measurements.front_distance);
        ret_right = ultrasonic_measure(&right_sensor, 400, &current_measurements.right_distance);
        ret_left = ultrasonic_measure(&left_sensor, 400, &current_measurements.left_distance);

        // Handle measurement errors by setting a sentinel value
        if (ret_front != ESP_OK)
        {
            ESP_LOGW(TAG, "Front sensor measurement failed: %s", esp_err_to_name(ret_front));
            current_measurements.front_distance = 10; // Use a large value to indicate an error/infinity
        }
        if (ret_right != ESP_OK)
        {
            ESP_LOGW(TAG, "Right sensor measurement failed: %s", esp_err_to_name(ret_right));
            current_measurements.right_distance = 10;
        }
        if (ret_left != ESP_OK)
        {
            ESP_LOGW(TAG, "Left sensor measurement failed: %s", esp_err_to_name(ret_left));
            current_measurements.left_distance = 10;
        }

        // Convert raw readings to centimeters
        current_measurements.front_distance *= 100.0f;
        current_measurements.right_distance *= 100.0f;
        current_measurements.left_distance *= 100.0f;

        // Send the complete data structure to the queue.
        // This will block if the queue is full, waiting for the consumer.
        if (xQueueSend(sensor_data_queue, &current_measurements, pdMS_TO_TICKS(50)) != pdTRUE)
        {
            ESP_LOGW(TAG, "Queue is full.");
        }
        else
        {
            ESP_LOGI(TAG, "Sent to queue -> R: %.2f, F: %.2f, L: %.2f",
                     current_measurements.right_distance, current_measurements.front_distance, current_measurements.left_distance);
        }

        // Wait before taking the next set of measurements
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}



// --- PUBLIC FUNCTIONS (Implementation) ---

esp_err_t init_control_ipc(void)
{
    // Create a queue to hold up to QUEUE_LENGTH items of size sensor_data_t.
    sensor_data_queue = xQueueCreate(QUEUE_LENGTH, sizeof(sensor_data_t));

    if (sensor_data_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create sensor data queue.");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Sensor data queue created successfully.");
    return ESP_OK;
}

esp_err_t start_control_task(void)
{
    BaseType_t ret = xTaskCreate(
        control_task,   // Function that implements the task.
        "control_task", // Text name for the task.
        4096,           // Stack size in words (not bytes).
        NULL,           // Parameter passed into the task.
        4,              // Priority at which the task is created.
        &control_task_handle            // Used to pass out the created task's handle.
    );

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create control_task.");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "control_task created successfully.");
    return ESP_OK;
}

esp_err_t start_sensor_task(void)
{
    BaseType_t ret = xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, &sensor_task_handle);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create sensor_task.");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "sensor_task created successfully.");
    return ESP_OK;
}

esp_err_t start_memory_task(void)
{
    BaseType_t ret = xTaskCreate(sensor_task, "memory_task", 4096, NULL, 10, &sensor_task_handle);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create memory_task.");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "sensor_task created successfully.");
    return ESP_OK;
}

