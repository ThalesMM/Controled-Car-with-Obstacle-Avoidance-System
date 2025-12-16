#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include "esp_err.h"

/**
 * @brief Inicializa a interface ESP-NOW para recepção de dados.
 *
 * Esta função deve ser chamada uma vez, após a inicialização do Wi-Fi.
 * Ela configura o serviço ESP-NOW e registra o callback para receber
 * os comandos de movimento do controle remoto.
 *
 * @return
 *    - ESP_OK: Sucesso na inicialização.
 *    - ESP_FAIL: Falha em alguma etapa da inicialização.
 */
esp_err_t espnow_comm_init(void);

#endif // ESPNOW_COMM_H