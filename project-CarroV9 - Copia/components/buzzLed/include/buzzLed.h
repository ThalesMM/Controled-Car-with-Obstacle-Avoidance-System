#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <esp_err.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/ledc.h" // <-- INCLUIR BIBLIOTECA LEDC
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Inicializa os pinos GPIO para o LED e o Buzzer (com PWM).
 *
 * @return esp_err_t ESP_OK se sucesso, ou um código de erro se falhar.
 */
esp_err_t feedback_init(void);

/**
 * @brief Define o estado do LED.
 *
 * @param on true para ligar, false para desligar.
 */
void feedback_buzz_set(bool on);
void feedback_ledR_set(bool on);
void feedback_ledL_set(bool on);

#endif // FEEDBACK_H