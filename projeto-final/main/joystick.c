#include "system_common.h"
#include "joystick.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// pinos conforme diagram.json
#define JOYSTICK_VERT_CHAN    ADC_CHANNEL_4   // GPIO 4
#define JOYSTICK_HOR_CHAN     ADC_CHANNEL_1   // GPIO 1
#define JOYSTICK_SW_PIN       GPIO_NUM_6

// quantas amostras a média móvel guarda
#define JOY_AVG_SAMPLES       16

static adc_oneshot_unit_handle_t adc1_handle;


void joystick_init(void) {
    // configura a unidade ADC1
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    // atenuação 12dB pra ler até ~3.3V
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK_VERT_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK_HOR_CHAN, &config));

    // botão como entrada com pull-up interno
    gpio_set_direction(JOYSTICK_SW_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(JOYSTICK_SW_PIN, GPIO_PULLUP_ONLY);
}


void joystick_read_task(void *pvParameters) {
    // buffers da média móvel
    int buf_x[JOY_AVG_SAMPLES] = {0};
    int buf_y[JOY_AVG_SAMPLES] = {0};
    int idx = 0;
    long sum_x = 0;
    long sum_y = 0;

    // pré-carga: faz umas leituras antes pra média começar estável
    int val_x, val_y;
    long cx = 0, cy = 0;
    for (int k = 0; k < 50; k++) {
        adc_oneshot_read(adc1_handle, JOYSTICK_HOR_CHAN, &val_x);
        adc_oneshot_read(adc1_handle, JOYSTICK_VERT_CHAN, &val_y);
        cx += val_x;
        cy += val_y;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    int rcx = cx / 50;
    int rcy = cy / 50;
    for (int i = 0; i < JOY_AVG_SAMPLES; i++) {
        buf_x[i] = rcx;
        buf_y[i] = rcy;
        sum_x += rcx;
        sum_y += rcy;
    }

    int last_btn = 1;
    uint32_t last_press = 0;

    for (;;) {
        // leitura bruta
        int raw_x = 0;
        int raw_y = 0;
        adc_oneshot_read(adc1_handle, JOYSTICK_HOR_CHAN, &raw_x);
        adc_oneshot_read(adc1_handle, JOYSTICK_VERT_CHAN, &raw_y);

        // janela deslizante: tira a leitura mais velha, coloca a nova
        sum_x -= buf_x[idx];
        sum_y -= buf_y[idx];
        buf_x[idx] = raw_x;
        buf_y[idx] = raw_y;
        sum_x += raw_x;
        sum_y += raw_y;

        idx++;
        if (idx >= JOY_AVG_SAMPLES) idx = 0;

        int sx = sum_x / JOY_AVG_SAMPLES;
        int sy = sum_y / JOY_AVG_SAMPLES;

        // botão com debounce simples por tempo
        int btn = gpio_get_level(JOYSTICK_SW_PIN);
        bool trig = false;

        if (last_btn == 1 && btn == 0) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if ((now - last_press) < 500 && (now - last_press) > 50) trig = true;
            last_press = now;
        }
        last_btn = btn;

        // grava na estrutura global
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            global_data.joy_x_raw = sx;
            global_data.joy_y_raw = sy;
            global_data.btn_pressed = (btn == 0);

            if (trig) global_data.calibration_trigger = true;

            xSemaphoreGive(data_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}