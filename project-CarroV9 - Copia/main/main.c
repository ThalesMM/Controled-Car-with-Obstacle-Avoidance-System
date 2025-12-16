#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// --- INÍCIO DAS ADIÇÕES DE INCLUDES ---
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "espnow_comm.h" // Inclui o header do nosso novo componente de comunicação
// --- FIM DAS ADIÇÕES DE INCLUDES ---

#include "ultrasonico.h"
#include "motor.h"
#include "control.h"
#include "buzzLed.h"

static const char *TAG = "MAIN";

// Handles globais (declarados em control.h ou aqui)
extern TaskHandle_t control_task_handle;
extern TaskHandle_t sensor_task_handle;

// --- INÍCIO DA FUNÇÃO ADICIONADA ---
/**
 * @brief Inicializa o NVS e o Wi-Fi em modo Estação.
 * Esta etapa é um pré-requisito obrigatório para usar o ESP-NOW.
 */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi inicializado com sucesso.");
}
// --- FIM DA FUNÇÃO ADICIONADA ---

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando o sistema do carro...");

    // --- INÍCIO DAS CHAMADAS DE FUNÇÃO ADICIONADAS ---
    // 1. Inicializa o Wi-Fi, que é necessário para o ESP-NOW funcionar.
    wifi_init();

    // 2. Inicializa o componente de comunicação ESP-NOW como receptor.
    if (espnow_comm_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Erro ao inicializar o ESP-NOW. Parando.");
        return;
    }
    ESP_LOGI(TAG, "Componente ESP-NOW inicializado como receptor.");
    // --- FIM DAS CHAMADAS DE FUNÇÃO ADICIONADAS ---

    // === Inicializações (SEU CÓDIGO ORIGINAL - SEM MUDANÇAS) ===
    if (ultrasonic_setup() != ESP_OK)
    {
        ESP_LOGE(TAG, "Erro ultrassom");
        return;
    }
    ESP_LOGI(TAG, "Sensores ultrassônicos inicializados");

    if (setup_hardware() != ESP_OK)
    {
        ESP_LOGE(TAG, "Erro motores");
        return;
    }
    ESP_LOGI(TAG, "Motores inicializados");

    if (feedback_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Erro LED");
        return;
    }
    ESP_LOGI(TAG, "LED de feedback inicializado");

    if (init_control_ipc() != ESP_OK)
    {
        ESP_LOGE(TAG, "Erro IPC");
        return;
    }
    ESP_LOGI(TAG, "Fila de sensores inicializada"); // Alterei o log para ser mais específico

    // === INÍCIO DAS TAREFAS ===
    if (start_sensor_task() != ESP_OK)
    {
        ESP_LOGE(TAG, "Erro sensor_task");
        return;
    }
    if (start_control_task() != ESP_OK)
    {
        ESP_LOGE(TAG, "Erro control_task");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(200));

    if (control_task_handle == NULL || sensor_task_handle == NULL)
    {
        ESP_LOGE(TAG, "Handles de task inválidos!");
        return;
    }

    ESP_LOGI(TAG, "Tasks criadas: control=%p, sensor=%p", control_task_handle, sensor_task_handle);

    // === MONITORAMENTO (SEU CÓDIGO ORIGINAL - SEM MUDANÇAS) ===
    TaskHandle_t tasks[] = {control_task_handle, sensor_task_handle};
    const char *names[] = {"control_task", "sensor_task"};
    const size_t num_tasks = 2;
    const uint32_t interval_ms = 5000;

    ESP_LOGI(TAG, "Monitoramento de memória ativado");

    // Loop infinito de monitoramento
    while (1)
    {
        // ... (seu excelente código de monitoramento de HEAP e STACK permanece aqui, sem alterações) ...
        uint32_t free_heap = xPortGetFreeHeapSize();
        ESP_LOGI(TAG, "----------------------------------------");
        ESP_LOGW(TAG, "HEAP Livre: %lu bytes", free_heap);
        for (size_t i = 0; i < num_tasks; i++)
        {
            UBaseType_t hwm = uxTaskGetStackHighWaterMark(tasks[i]);
            ESP_LOGW(TAG, "STACK HWM -> [%s] %lu bytes livres", names[i], (unsigned long)hwm * sizeof(StackType_t));
        }
        ESP_LOGI(TAG, "----------------------------------------");
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }
}