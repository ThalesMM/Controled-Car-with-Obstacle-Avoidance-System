idf_component_register(SRCS "main.c"
                      INCLUDE_DIRS "."
                      REQUIRES 
                                # Seus componentes customizados
                                motor
                                ultrasonico
                                buzzLed
                                control
                                espnow_comm

                                # Componentes de sistema necessários para o Wi-Fi e ESP-NOW
                                nvs_flash    # Adicionado para nvs_flash_init()
                                esp_netif    # Corrigido o typo
                                esp_event    # Adicionado para o loop de eventos
                                esp_wifi
)