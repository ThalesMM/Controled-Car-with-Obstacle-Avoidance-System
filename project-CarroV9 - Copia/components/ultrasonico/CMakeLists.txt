idf_component_register(SRCS "ultrasonico.c"
                    REQUIRES esp_driver_gpio esp_driver_ledc esp-idf-lib__ultrasonic
                    INCLUDE_DIRS "include" )
