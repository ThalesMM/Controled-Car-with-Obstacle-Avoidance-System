#include "buzzLed.h"

// --- Configurações do LED ---
#define LED_PINR GPIO_NUM_13
#define LED_PINL GPIO_NUM_4

// --- Configurações do Buzzer com LEDC (PWM) ---
#define BUZZER_PIN GPIO_NUM_32
#define LEDC_TIMER LEDC_TIMER_1 // Usar um timer diferente do dos motores
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_2     // Usar um canal diferente dos motores
#define LEDC_CHANNEL LEDC_CHANNEL_3     // Usar um canal diferente dos motores
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // Resolução de 10-bit (0-1023)
#define LEDC_DUTY (512)                 // 50% duty cycle (liga metade do tempo, desliga metade)
#define LEDC_FREQUENCY (2000)           // Frequência do som em Hz (ex: 2kHz)

esp_err_t feedback_init(void)
{
    // 1. Configurar o pino do LED
    gpio_reset_pin(LED_PINL);
    gpio_set_direction(LED_PINL, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PINL, 0);
    gpio_reset_pin(LED_PINR);
    gpio_set_direction(LED_PINR, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PINR, 0);

    // 2. Configurar o Timer do LEDC para o Buzzer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 3. Configurar o Canal do LEDC para o Buzzer
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BUZZER_PIN,
        .duty = 0, // Inicia desligado
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    return ESP_OK;
}

void feedback_buzz_set(bool on)
{
    // Controla o Buzzer via PWM
    if (on)
    {
        // Liga o buzzer com 50% de duty cycle para gerar som
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }
    else
    {
        // Desliga o buzzer (duty cycle = 0)
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }
}
void feedback_ledR_set(bool on)
{
    // Controla o LED
    gpio_set_level(LED_PINR, on ? 1 : 0);
    

}
void feedback_ledL_set(bool on)
{
    // Controla o LED
    gpio_set_level(LED_PINL, on ? 1 : 0);

}


