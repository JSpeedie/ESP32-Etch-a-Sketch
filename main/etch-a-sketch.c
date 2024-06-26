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
/* This is for GPIO pin 33 */
static const adc_channel_t potentiometer_horiz_channel = ADC_CHANNEL_5;
/* This is for GPIO pin 39 */
static const adc_channel_t potentiometer_vert_channel = ADC_CHANNEL_3;
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
#define SPI_CS_PIN_NUM   32
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


/* Returns a value from (-1, 1) depending on the size of the value x. */
double fast_sigmoid(double x) {
    return x / (1 + fabs(x));
}


/* Returns a value from (0, 1) depending on the size of the value x. */
double fast_sigmoid2(double x) {
    return 0.5 * (x / (1 + fabs(x)) + 1);
}


/* Returns a value from [0, 1] depending on the size of the value x.
 * This function is used to handle noise in input, returning a factor
 * that would shrink a change if the change is small. */
double certainty_factor(double x) {
    /* return (1 / (-pow(fabs(x)-0.25, 4) - 1)) + 1; // 1st Attempt */
    return (2 / (-pow(fabs(x)-0.25, 4) - 2)) + 1; // 2nd Attempt
}


void merge_sort(int *arr, int arr_size) {

    if (arr_size > 2) {
        /* Merge sort the two halfs of the array individually */
        merge_sort(&arr[0], (arr_size / 2));
        merge_sort(&arr[arr_size / 2], ((arr_size + 1) / 2));

        /* Zip together the two sorted arrays */
        int i_a = 0;
        int i_b = arr_size / 2;
        int temp[arr_size];

        for (int i = 0; i < arr_size; i++) {
            if (i_b >= arr_size) {
                temp[i] = arr[i_a];
                i_a++;
                continue;
            }
            if (i_a >= arr_size / 2) {
                temp[i] = arr[i_b];
                i_b++;
                continue;
            }
            if (arr[i_a] <= arr[i_b]) {
                temp[i] = arr[i_a];
                i_a++;
            } else {
                temp[i] = arr[i_b];
                i_b++;
            }
        }

        /* Copy the contents of the temp array back into the original array */
        for (int i = 0; i < arr_size; i++) {
            arr[i] = temp[i];
        }

    } else if (arr_size == 2) {
        if (arr[0] <= arr[1]) {
            return;
        } else {
            int t = arr[0];
            arr[0] = arr[1];
            arr[1] = t;
            return;
        }
    } else {
        return;
    }
}


double standard_deviation(int *arr, int arr_size) {

    /* Calculate the mean */
    double mean = 0;
    for (int i = 0; i < arr_size; i++) {
        mean += arr[i];
    }
    mean /= (double) arr_size;

    double sd = 0;
    for (int i = 0; i < arr_size; i++) {
        sd += fabs(((double) arr[i]) - mean);
    }
    sd /= (double) arr_size;

    return sd;
}


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
    uint8_t prev_cursor_x = game_state.cursor_x;
    uint8_t prev_cursor_y = game_state.cursor_y;
    int screen_clear = 0;

    while (1) {
        /* Update the lastWakeTime variable to have the current time */
        lastWakeTime = xTaskGetTickCount();

        /* Update the stored location of the ball */
        if (xSemaphoreTake(input_semaphore, portMAX_DELAY) == pdTRUE) {
            prev_cursor_x = cursor_x;
            prev_cursor_y = cursor_y;
            cursor_x = game_state.cursor_x;
            cursor_y = game_state.cursor_y;
            xSemaphoreGive(input_semaphore);
        }

        /* Calculate the length of the line between the previous cursor
         * location and the current cursor location */
        int dx = cursor_x - prev_cursor_x;
        int dy = cursor_y - prev_cursor_y;
        /* a^2 + b^2 = c^2 -> c = sqrt(a^2 + b^2) */
        int line_length = sqrt((dx * dx) + (dy * dy));

        int spot_x = prev_cursor_x;
        int spot_y = prev_cursor_y;
        int d_spot_x = round((double) dx / (double) line_length);
        int d_spot_y = round((double) dy / (double) line_length);

        /* Paint white pixels at the spots along the line. This is done
         * to avoid gaps when the cursor is moved quickly */
        for (int i = 0; i < line_length; i++) {
            /* Update oled_screen array to have pixel at the specified location
             * painted white. Because the display is technically 128 by 128
             * pixels but is represented in memory as 64 columns by 128 rows
             * where each element in a column is a pair representing two
             * horizontally adjacent pixels, this is how we set the pixel at
             * (x, y) on. */
            if (spot_x % 2 == 0) {
                oled_screen[(64 * spot_y) + (spot_x / 2)] |= 0xf0;
            } else {
                oled_screen[(64 * spot_y) + (spot_x / 2)] |= 0x0f;
            }
            spot_x += d_spot_x;
            spot_y += d_spot_y;
        }

        /* UNpaint the final spot (the new location of the cursor), like
         * the real toy */
        if (cursor_x % 2 == 0) {
            oled_screen[(64 * cursor_y) + (cursor_x / 2)] &= 0x0f;
        } else {
            oled_screen[(64 * cursor_y) + (cursor_x / 2)] &= 0xf0;
        }

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
    int pot_sample_per_tick = 35;
    int pot_prev_tick_samples_saved = 3; /* I.e. the number of points in our Moving Average Filter */
    int pot_prev_tick_index = 0;
    double pot_h_prev_tick[pot_prev_tick_samples_saved];
    double pot_v_prev_tick[pot_prev_tick_samples_saved];
    int pot_h_samples[pot_sample_per_tick];
    int pot_v_samples[pot_sample_per_tick];

    /* Zero out the previous tick array before we get to the loop
     * so the array isn't initialized with garbage data */
    for (int i = 0; i < pot_prev_tick_samples_saved; i++) {
        pot_h_prev_tick[i] = 0;
        pot_v_prev_tick[i] = 0;
    }

    /* 1. Configure the ADC unit. The ESP32 has 2 ADC units, and each
     * ADC-capable pin is assigned to one of the units. The 2 pins used in this
     * program are both on ADC unit 1 */
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t adc_unit_cfg = {
        .unit_id = ADC_UNIT_1,  /* Pins 33 and 39 use ADC1, channels 5 and 3 */
        .ulp_mode = ADC_ULP_MODE_DISABLE, /* Do not use Ultra Low Power mode */
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_unit_cfg, &adc1_handle));

    /* 2. Configure the individual ADC oneshots. The config below will be used
     * to configure both the horizontal and vertical potentiometers as they
     * will use the same bitwidth and attenuation */
    adc_oneshot_chan_cfg_t adc_chan_config = {
        .bitwidth = ADC_BITWIDTH_10,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, potentiometer_horiz_channel, &adc_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, potentiometer_vert_channel, &adc_chan_config));
    /* }}} */

    while (1) {
        /* Update the lastWakeTime variable to have the current time */
        lastWakeTime = xTaskGetTickCount();

        /* Read a few ADC values from each potentiometer since the ADC unit is
         * quite unstable */
        for (int a = 0; a < pot_sample_per_tick; a++) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, \
                potentiometer_horiz_channel, &pot_h_samples[a]));

            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, \
                potentiometer_vert_channel, &pot_v_samples[a]));
        }

        /* Sort the samples from this tick (so we can find the median) */
        merge_sort(&pot_h_samples[0], pot_sample_per_tick);
        merge_sort(&pot_v_samples[0], pot_sample_per_tick);

        /* double h_sd = standard_deviation(&pot_h_samples[0], pot_sample_per_tick); */
        /* double v_sd = standard_deviation(&pot_v_samples[0], pot_sample_per_tick); */
        /* printf("%g, %g\n", h_sd, v_sd); */

        /* Take a mean of several values around the median from the samples
         * collected this tick (call this the mean-median) and save it in the
         * saved mean-medians array */
        int values_around_median = 3;
        pot_h_prev_tick[pot_prev_tick_index] = 0;
        pot_v_prev_tick[pot_prev_tick_index] = 0;

        for (int i = 0; i < values_around_median; i++) {
            pot_h_prev_tick[pot_prev_tick_index] += pot_h_samples[(pot_sample_per_tick / 2) - (values_around_median / 2) + i];
            pot_v_prev_tick[pot_prev_tick_index] += pot_v_samples[(pot_sample_per_tick / 2) - (values_around_median / 2) + i];
        }

        pot_h_prev_tick[pot_prev_tick_index] /= (double) values_around_median;
        pot_v_prev_tick[pot_prev_tick_index] /= (double) values_around_median;

        /* Increase the previous tick array index, resetting to 0 if we hit
         * the end of the array */
        pot_prev_tick_index++;
        if (pot_prev_tick_index >= pot_prev_tick_samples_saved) {
            pot_prev_tick_index = 0;
        }

        horiz_val = 0;
        vert_val = 0;

        /* Average the mean-median from this tick with the mean-medians saved
         * from previous ticks */
        for (int i = 0; i < pot_prev_tick_samples_saved; i++) {
            horiz_val += pot_h_prev_tick[i];
            vert_val += pot_v_prev_tick[i];
        }

        horiz_val /= (double) pot_prev_tick_samples_saved;
        vert_val /= (double) pot_prev_tick_samples_saved;

        if (xSemaphoreTake(input_semaphore, portMAX_DELAY) == pdTRUE) {
            /* 1024 / (128 - cursor_size) because we use a 10 bit ADC (2^10 =
             * 1024 distinct values) and we want to scale the 0-1023 range we
             * receive from the potentiometer to the range of pixels the ball
             * could occupy */
            double adc_to_screen_scale = 1024 / (128 - cursor_size);
            double temp_new_cursor_x = (horiz_val / adc_to_screen_scale);
            double temp_new_cursor_y = (vert_val / adc_to_screen_scale);

            /* Put the input through one final anti-noise filter. Here we use a
             * custom made function to impart a smaller change on the cursor's
             * position if the perceived change (i.e. dx_c, dy_c) is small. For
             * instance, if the perceived change in the cursor's x position is
             * 0.5, this gives a certainty factor of ~0.004, greatly reducing
             * how much the cursor's x position will be shifted. We don't want
             * large perceived changes to be affected however, and this is
             * reflected in the function. For a perceived change of 3, we get a
             * certainty factor of ~0.983, hardly changing how the cursor will
             * be moved. */

            /* We add 0.5 to the current cursor position because it better
             * represents the cursor's position as being in-the-middle of
             * the pixel */
            double dx_c = temp_new_cursor_x - (double) (game_state.cursor_x + 0.5);
            game_state.cursor_x += round(certainty_factor(dx_c) * dx_c);

            double dy_c = temp_new_cursor_y - (double) (game_state.cursor_y + 0.5);
            game_state.cursor_y += round(certainty_factor(dy_c) * dy_c);

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

    printf("Etch-a-Sketch application is starting!\n");

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
