#include "servo.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdlib.h>
#include "system_common.h"


// pinos dos servos 
#define SERVO_X_PIN             15
#define SERVO_Y_PIN             2

// configuração do LEDC pra PWM dos servos
#define LEDC_TIMER_SERVO        LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_SERVO         50                  // 50Hz = 20ms (padrão servo)
#define LEDC_RES_SERVO          LEDC_TIMER_13_BIT   // 0..8191
#define LEDC_CHANNEL_X          LEDC_CHANNEL_0
#define LEDC_CHANNEL_Y          LEDC_CHANNEL_1

// fator de suavização (quanto menor, mais "macio" o movimento)
#define SERVO_SMOOTH_FACTOR     0.60f
// zona morta no centro do joystick (evita tremor com o stick parado)
#define JOY_DEADZONE            50
#define ADC_MIN_READ            100
#define ADC_MAX_READ            4000

//   LIMITADORES MANUAIS DOS SERVOS (em %)
#define SERVO_X_MIN_PERCENT     0     // limite inferior do servo X
#define SERVO_X_MAX_PERCENT     100   // limite superior do servo X
#define SERVO_Y_MIN_PERCENT     0     // limite inferior do servo Y
#define SERVO_Y_MAX_PERCENT     100   // limite superior do servo Y

// faixa absoluta de PWM que cobre o curso total do servo (~0° a 180°)
// 50Hz @ 13 bits: 1ms = 410, 2ms = 819
// dá uma margem pra não forçar a engrenagem nos extremos
#define SERVO_PWM_ABS_MIN       410   // ~0°
#define SERVO_PWM_ABS_MAX       820   // ~180°


// faz interpolação linear entre dois intervalos e trava nos limites
static int map_value(int value, int in_min, int in_max, int out_min, int out_max) {
    long result = (long)(value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if (out_min < out_max) {
        if (result < out_min) return out_min;
        if (result > out_max) return out_max;
    } else {
        if (result < out_max) return out_max;
        if (result > out_min) return out_min;
    }
    return (int)result;
}

// se o joystick estiver perto do centro, força o valor central
static int apply_deadzone(int raw_val) {
    int center = (ADC_MAX_READ + ADC_MIN_READ) / 2;
    if (abs(raw_val - center) < JOY_DEADZONE) return center;
    return raw_val;
}

// converte a porcentagem definida nos #define em PWM real
static int percent_to_pwm(int percent) {
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;
    return SERVO_PWM_ABS_MIN +
           ((SERVO_PWM_ABS_MAX - SERVO_PWM_ABS_MIN) * percent) / 100;
}


void servos_init(void) {
    // timer compartilhado entre os dois canais
    ledc_timer_config_t timer_servo = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER_SERVO,
        .duty_resolution = LEDC_RES_SERVO,
        .freq_hz         = LEDC_FREQ_SERVO,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_servo));

    // canal do servo X
    ledc_channel_config_t ch_x = {
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL_X,
        .timer_sel  = LEDC_TIMER_SERVO,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = SERVO_X_PIN,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_x));

    // canal do servo Y
    ledc_channel_config_t ch_y = {
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL_Y,
        .timer_sel  = LEDC_TIMER_SERVO,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = SERVO_Y_PIN,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_y));

    // aplica os limites definidos pelos #define direto no estado global
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        global_data.servo_min_x = percent_to_pwm(SERVO_X_MIN_PERCENT);
        global_data.servo_max_x = percent_to_pwm(SERVO_X_MAX_PERCENT);
        global_data.servo_min_y = percent_to_pwm(SERVO_Y_MIN_PERCENT);
        global_data.servo_max_y = percent_to_pwm(SERVO_Y_MAX_PERCENT);
        xSemaphoreGive(data_mutex);
    }
}


void servo_control_task(void *pvParameters) {
    system_data_t local;

    // guarda a posição atual em float (precisa pro filtro suavizar bem)
    static float current_x_f = 0.0f;
    static float current_y_f = 0.0f;
    bool initialized = false;

    printf(">>> servo_control_task iniciada\n");

    for (;;) {
        // pega a leitura mais recente do joystick
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            local = global_data;
            xSemaphoreGive(data_mutex);
        }

        // na primeira passada, centraliza os servos
        if (!initialized) {
            current_x_f = (float)(local.servo_min_x + local.servo_max_x) / 2.0f;
            current_y_f = (float)(local.servo_min_y + local.servo_max_y) / 2.0f;
            initialized = true;
        }

        // tira o ruído da zona central
        int joy_x_clean = apply_deadzone(local.joy_x_raw);
        int joy_y_clean = apply_deadzone(local.joy_y_raw);

        // converte 0..4095 do ADC pra faixa de PWM do servo
        int target_x = map_value(joy_x_clean, ADC_MIN_READ, ADC_MAX_READ,
                                 local.servo_max_x, local.servo_min_x);
        int target_y = map_value(joy_y_clean, ADC_MIN_READ, ADC_MAX_READ,
                                 local.servo_max_y, local.servo_min_y);

        // filtro exponencial: aproxima da posição alvo aos poucos
        current_x_f += ((float)target_x - current_x_f) * SERVO_SMOOTH_FACTOR;
        current_y_f += ((float)target_y - current_y_f) * SERVO_SMOOTH_FACTOR;

        // manda pro hardware
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, (int)current_x_f);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, (int)current_y_f);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y);

        // 50Hz = mesmo período do PWM do servo
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}