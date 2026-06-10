#include "status.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "system_common.h"
#include <stdio.h>

// LED externo conectado ao GPIO 10 
#define STATUS_LED_PIN      GPIO_NUM_10

// quantas vezes o LED pisca antes de ficar aceso fixo
#define BOOT_BLINK_COUNT    5
#define BOOT_BLINK_MS       150


void status_init(void) {
    gpio_set_direction(STATUS_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(STATUS_LED_PIN, 0);
}


// pisca o LED algumas vezes pra indicar que o boot acabou,
// depois deixa aceso fixo enquanto o sistema tá rodando
static void boot_indicator(void) {
    for (int i = 0; i < BOOT_BLINK_COUNT; i++) {
        gpio_set_level(STATUS_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(BOOT_BLINK_MS));
        gpio_set_level(STATUS_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(BOOT_BLINK_MS));
    }
    // LED aceso = sistema pronto pra uso
    gpio_set_level(STATUS_LED_PIN, 1);
}


void status_task(void *pvParameters) {
    // dá um tempinho pras outras tasks subirem antes
    vTaskDelay(pdMS_TO_TICKS(300));

    boot_indicator();

    // marca como pronto
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        global_data.system_ready = true;
        xSemaphoreGive(data_mutex);
    }

    printf("=== Mesa Labirinto pronta ===\n");

    system_data_t snap;

    for (;;) {
        // tira uma cópia rápida dos dados pra não segurar o mutex
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            snap = global_data;
            xSemaphoreGive(data_mutex);
        }

        // log periódico no monitor serial
        printf("[STATUS] joy=(%4d,%4d) | limX=[%d-%d] limY=[%d-%d] | btn=%d\n",
               snap.joy_x_raw, snap.joy_y_raw,
               snap.servo_min_x, snap.servo_max_x,
               snap.servo_min_y, snap.servo_max_y,
               snap.btn_pressed ? 1 : 0);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}