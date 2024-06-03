set(srcs "etch-a-sketch.c")

idf_component_register(SRCS "${srcs}"
                    REQUIRES driver
                    REQUIRES esp_adc
                    PRIV_REQUIRES esp32-spi-ssd1327
                    INCLUDE_DIRS ".")
