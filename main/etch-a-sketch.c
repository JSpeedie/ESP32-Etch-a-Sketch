#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_heap_caps.h" // For DMA allocation
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "etch-a-sketch.h"

/* Component includes */
#include "esp32-spi-ssd1327.h"


/* Potentiometer Defines {{{ */
/* Going off  https://learn.adafruit.com/assets/111179 */
/* This is for GPIO pin 39 */
static const adc_channel_t potentiometer_horiz_channel = ADC_CHANNEL_3;
/* This is for GPIO pin 36 */
static const adc_channel_t potentiometer_vert_channel = ADC_CHANNEL_0;
/* }}} */

/* GPIO Defines {{{ */
/* Going off  https://learn.adafruit.com/assets/111179 */
#define GPIO_SCREEN_CLEAR_BUTTON_PIN_NUM  4
/* }}} */

/* SPI Defines {{{ */
/* Going off  https://learn.adafruit.com/assets/111179 */
#define SPI_MOSI_PIN_NUM 18
/* #define SPI_MISO_PIN_NUM 19 */ // Not used for the OLED
#define SPI_SCK_PIN_NUM  14
#define SPI_CS_PIN_NUM   15
#define DC_PIN_NUM   26
#define RST_PIN_NUM  25

/* SPI0 and SPI1 are reserved so we can choose between SPI2 and SPI3 which
 * are also referred to elsewhere as HSPI and VSPI respectively. Here we
 * commit to using HSPI */
#define SPI_HOST_TAG SPI2_HOST
/* }}} */

/* I2C Defines {{{ */
#define I2C_BUS_PORT 0
/* Going off  https://learn.adafruit.com/assets/111179 */
#define I2C_SDA_PIN_NUM 23
#define I2C_SCL_PIN_NUM 22
/* }}} */


SemaphoreHandle_t input_semaphore = NULL;
struct game_state game_state = {
    .cursor_x = 128.0/2,
    .cursor_y = 128.0/2,
};
double cursor_size = 1;


void paint_oled(void *arg) {
    /* SPI OLED + GPIO Button Initialization {{{ */
	/* 1. Configure the spi master bus */
    spi_bus_config_t spi_bus_cfg = {
        .miso_io_num = -1,
        .mosi_io_num = SPI_MOSI_PIN_NUM,
        .sclk_io_num = SPI_SCK_PIN_NUM,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));

    /* 2. Configure the spi device */
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 10 * 1000 * 1000,      // Clock out at 10 MHz
        .mode = 0,                               // SPI mode 0
        .spics_io_num = SPI_CS_PIN_NUM,          // CS pin
        .queue_size = 7,                         // We want to be able to queue 7 transactions at a time
    };

    spi_device_handle_t *oled_dev_handle = malloc(sizeof(spi_device_handle_t));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_TAG, &dev_cfg, oled_dev_handle));

    /* 3. Initialize the remaining GPIO pins */
    /* DC GPIO pin */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DC_PIN_NUM),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&io_conf);

    /* RST GPIO pin */
    gpio_config_t io_conf2 = {
        .pin_bit_mask = (1ULL << RST_PIN_NUM),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf2);

    /* Screen Clear Button GPIO pin */
    gpio_config_t io_conf3 = {
        .pin_bit_mask = (1ULL << GPIO_SCREEN_CLEAR_BUTTON_PIN_NUM),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&io_conf3);

    /* 4. Create SSD1327 struct for use of the spi_oled functions */
    struct spi_ssd1327 *spi_ssd1327 = malloc(sizeof(struct spi_ssd1327));
    spi_ssd1327->dc_pin_num = DC_PIN_NUM;
    spi_ssd1327->rst_pin_num = RST_PIN_NUM;
    spi_ssd1327->spi_handle = oled_dev_handle;

    spi_oled_init(spi_ssd1327);
    /* }}} */

    uint8_t *oled_screen = heap_caps_malloc(sizeof(uint8_t) * 128 * 64, MALLOC_CAP_DMA);

    /* Paint the whole screen black */
    for (int i = 0; i < 128; i++) {
        for (int j = 0; j < 64; j++) {
            oled_screen[(64 * i) + j] = 0x00;
        }
    }
    spi_oled_draw_image(spi_ssd1327, oled_screen);

    /* Set the frequency of the loop in this function to 3 ticks */
    const TickType_t taskFrequency = 3;
    TickType_t lastWakeTime;
    uint8_t cursor_x = game_state.cursor_x;
    uint8_t cursor_y = game_state.cursor_y;
    int screen_clear = 0;

    while (1) {
        /* Update the lastWakeTime variable to have the current time */
        lastWakeTime = xTaskGetTickCount();

        /* Update the stored location of the ball */
        if (xSemaphoreTake(input_semaphore, portMAX_DELAY) == pdTRUE) {
            cursor_x = game_state.cursor_x;
            cursor_y = game_state.cursor_y;
            xSemaphoreGive(input_semaphore);
        }

        /* Update oled_screen array to have pixel at the new location of the
         * cursor painted white. Because the display is technically 128 by 128
         * pixels but is represented in memory as 64 columns by 128 rows where
         * each element in a column is a pair representing two horizontally
         * adjacent pixels, this is how we set the pixel at (x, y) on. */
        oled_screen[(64 * cursor_y) + (cursor_x / 2)] |= 0xf0 >> (cursor_x % 2);

        /* If the screen clear button was pressed, erase the screen */
        screen_clear = gpio_get_level(GPIO_SCREEN_CLEAR_BUTTON_PIN_NUM);
        if (screen_clear == 1) {
            /* Paint the whole screen black */
            for (int i = 0; i < 128; i++) {
                for (int j = 0; j < 64; j++) {
                    oled_screen[(64 * i) + j] = 0x00;
                }
            }
        }

        spi_oled_draw_image(spi_ssd1327, oled_screen);

        /* Delay such that this loop executes every 'taskFrequency' ticks */
        vTaskDelayUntil(&lastWakeTime, taskFrequency);
    }
}


/** Take a struct containing both pointers to where the 9 DOF sensor data
 * is stored (so it can update it) and game state data so it can adjust
 * it as well */
void get_input(void *arg) {
    /* Set the frequency of the loop in this function to 3 ticks */
    const TickType_t taskFrequency = 3;
    TickType_t lastWakeTime;
    double horiz_val = 0;
    double vert_val = 0;

    /* Potentiometer Initialization {{{ */
    int pot_sample_per_tick = 3;
    int pot_prev_tick_samples_saved = 20;
    int pot_num_saved_values = pot_sample_per_tick * pot_prev_tick_samples_saved;
    int pot_h_adc_values[pot_num_saved_values];
    int pot_v_adc_values[pot_num_saved_values];

    /* 1. Configure the ADC unit. The ESP32 has 2 ADC units, and each
     * ADC-capable pin is assigned to one of the units. The 2 pins used in this
     * program are both on ADC unit 1 */
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t adc_unit_cfg = {
        .unit_id = ADC_UNIT_1,  /* Pins 39 and 36 use ADC1, channels 3 and 0 */
        .ulp_mode = ADC_ULP_MODE_DISABLE, /* Do not use Ultra Low Power mode */
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_unit_cfg, &adc1_handle));

    /* 2. Configure the individual ADC oneshots. The config below will be used
     * to configure both the horizontal and vertical potentiometers as they
     * will use the same bitwidth and attenuation */
    adc_oneshot_chan_cfg_t adc_chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, potentiometer_horiz_channel, &adc_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, potentiometer_vert_channel, &adc_chan_config));
    /* }}} */

    while (1) {
        /* Update the lastWakeTime variable to have the current time */
        lastWakeTime = xTaskGetTickCount();

        /* New tick, shift all stored values in the potentiometer value arrays
         * back by as many spots as we sample per tick to make room for the
         * samples of this tick */
        for (int i = pot_num_saved_values - 1; i > pot_sample_per_tick - 1; i--) {
            pot_h_adc_values[i] = pot_h_adc_values[i - pot_sample_per_tick];
            pot_v_adc_values[i] = pot_v_adc_values[i - pot_sample_per_tick];
        }
        /* Read a few ADC values from each potentiometer since the ADC unit is
         * quite unstable */
        for (int a = 0; a < pot_sample_per_tick; a++) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, \
                potentiometer_horiz_channel, \
                &pot_h_adc_values[a]));

            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, \
                potentiometer_vert_channel, \
                &pot_v_adc_values[a]));
        }

        /* Average the horizontal and vertical potentiometer values
         * over the most recent samples */
        horiz_val = 0;
        vert_val = 0;

        for (int i = 0; i < pot_num_saved_values; i++) {
            horiz_val += pot_h_adc_values[i];
            vert_val += pot_v_adc_values[i];
        }

        horiz_val /= pot_num_saved_values;
        vert_val /= pot_num_saved_values;

        if (xSemaphoreTake(input_semaphore, portMAX_DELAY) == pdTRUE) {
            /* 32.2519 = 4096 / (128 - cursor_size). We want to scale the 0-4095 range
             * we receive from the potentiometer to the range of pixels the
             * ball could occupy */
            double temp_new_cursor_x = (horiz_val / (double) 32.2519685039);
            double temp_new_cursor_y = (vert_val / (double) 32.2519685039);
            /* Only change the cursor x or y if the new cursor x or y location
             * is more than 50% of a pixel into the next pixel. This is to help
             * with drawing straight lines */
            /* if (fabs(temp_new_cursor_x - game_state.cursor_x + 0.5) > 0.5) { */
                game_state.cursor_x = round(temp_new_cursor_x);
            /* } */
            /* if (fabs(temp_new_cursor_y - game_state.cursor_y + 0.5) > 0.5) { */
                game_state.cursor_y = round(temp_new_cursor_y);
            /* } */

            printf("new cursor (x, y) = (%d, %d)\n", game_state.cursor_x, game_state.cursor_y);

            /* Make sure the ball's position respects the bounds */
            if (game_state.cursor_x < 0) {
                game_state.cursor_x = 0;
            } else if (game_state.cursor_x > 127 - (cursor_size - 1)) {
                game_state.cursor_x = 127 - (cursor_size - 1);
            }
            if (game_state.cursor_y < 0) {
                game_state.cursor_y = 0;
            } else if (game_state.cursor_y > 127 - (cursor_size - 1)) {
                game_state.cursor_y = 127 - (cursor_size - 1);
            }

            xSemaphoreGive(input_semaphore);
        }

        /* Delay such that this loop executes every 'taskFrequency' ticks */
        vTaskDelayUntil(&lastWakeTime, taskFrequency);
    }
}


void app_main(void) {
    /* Create the variables that will be shared between
     * the paint_oled and the get_input tasks */
    cursor_size = 1;

    vSemaphoreCreateBinary(input_semaphore);
    if (input_semaphore == NULL) {
        printf("ERROR: creating semaphore!\n");
    }

    TaskHandle_t paint_oled_task;
    TaskHandle_t get_input_task;

    /* TODO: The stack allocations for each of these tasks is almost certainly
     * excessive */
    xTaskCreatePinnedToCore(paint_oled, "paint_oled", 20480, \
        (void *)NULL, 10, &paint_oled_task, 0);
    xTaskCreatePinnedToCore(get_input, "get_input", 20480, \
        (void *)NULL, 10, &get_input_task, 1);
}
