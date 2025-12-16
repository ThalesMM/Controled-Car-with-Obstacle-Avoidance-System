#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_mac.h"

// Inclui o header do próprio componente
#include "espnow_comm.h"
// Inclui o header do componente de controle para poder chamar sua função
#include "control.h"

// Tag para os logs deste componente
static const char *TAG = "ESPNOW_RECEIVER";

/**
 * @brief Callback executado quando um pacote ESP-NOW é recebido.
 *
 * Esta função é o coração do receptor. Ela é chamada automaticamente
 * pelo driver do ESP-NOW em um contexto de alta prioridade.
 *
 * @param recv_info Informações sobre o pacote recebido, incluindo o MAC do remetente.
 * @param data Ponteiro para os dados recebidos.
 * @param len Comprimento dos dados recebidos.
 */
static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    // Verificações de segurança
    if (recv_info == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Erro na recepção: dados inválidos.");
        return;
    }

    // Verifica se o tamanho do dado recebido corresponde ao tamanho do nosso tipo de comando
    if (len != sizeof(joystick_state_t))
    {
        ESP_LOGW(TAG, "Pacote recebido com tamanho incorreto: %d bytes", len);
        return;
    }

    // Log para depuração: informa de quem recebemos o pacote
    ESP_LOGI(TAG, "Dado recebido " MACSTR, MAC2STR(recv_info->src_addr));

    // Converte os dados brutos recebidos para o nosso tipo de comando
    //joystick_state_t received_command = *(joystick_state_t *)data;

    // Converte os dados brutos recebidos para o nosso tipo de comando
    joystick_state_t received_command;
    memcpy(&received_command, data, sizeof(joystick_state_t));

    // Ação principal: "Injeta" o comando recebido no componente de controle
    // chamando a função pública que criamos para isso.
    control_set_remote_command(received_command);

    //ESP_LOGI(TAG, "Comando do joystick atualizado para: %d", received_command);
}

// --- Implementação da Função Pública ---

esp_err_t espnow_comm_init(void)
{
    ESP_LOGI(TAG, "Inicializando ESP-NOW como receptor...");

    // Inicializa o serviço ESP-NOW. Deve ser chamado após a inicialização do Wi-Fi.
    if (esp_now_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao inicializar ESP-NOW.");
        return ESP_FAIL;
    }

    // Registra a função de callback que será chamada quando um pacote chegar.
    if (esp_now_register_recv_cb(esp_now_recv_cb) != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao registrar o callback de recepção.");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ESP-NOW inicializado com sucesso como receptor.");
    return ESP_OK;
}