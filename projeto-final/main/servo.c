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

// fator de suavização
#define SERVO_SMOOTH_FACTOR     0.20f   // mais lento = mais suave sob carga

// zona morta do joystick — aumentada pra evitar micro-movimentos de ADC
#define JOY_DEADZONE            150
#define ADC_MIN_READ            100
#define ADC_MAX_READ            4000

// LIMITADORES MANUAIS DOS SERVOS (em %)
#define SERVO_X_MIN_PERCENT     30
#define SERVO_X_MAX_PERCENT     80
#define SERVO_Y_MIN_PERCENT     30
#define SERVO_Y_MAX_PERCENT     80

// POSIÇÃO INICIAL DOS SERVOS (em %, 0 a 100)
// 50 = centro exato. Ajuste até a mesa ficar nivelada.
// Aumente se a mesa cai pra frente/esquerda, diminua se cai pra trás/direita.
#define SERVO_X_START_PERCENT  100
#define SERVO_Y_START_PERCENT  100

// faixa de PWM (~0° a 180°) — 50Hz @ 13 bits
#define SERVO_PWM_ABS_MIN       410   // ~0°
#define SERVO_PWM_ABS_MAX       820   // ~180°

// ── Gate de movimento ─────────────────────────────────────────────────────────
// Threshold em ticks de duty cycle para considerar que houve movimento real.
// Com faixa 410-820 (410 ticks = 180°), 1 tick ≈ 0,44°. Valor 3 = ~1,3°.
#define SERVO_MOVE_THRESHOLD    5

// Quantos ciclos de 20ms enviar após parar, para o servo assentar na posição
// antes de desligar o sinal completamente.
#define SERVO_SETTLE_CYCLES     3

// Snap: se o filtro estiver a menos de N ticks do target, força convergência
// imediata em vez de continuar oscilando em float.
#define SERVO_SNAP_THRESHOLD    3.0f
// ─────────────────────────────────────────────────────────────────────────────


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

static int apply_deadzone(int raw_val) {
    int center = (ADC_MAX_READ + ADC_MIN_READ) / 2;
    if (abs(raw_val - center) < JOY_DEADZONE) return center;
    return raw_val;
}

static int percent_to_pwm(int percent) {
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;
    return SERVO_PWM_ABS_MIN +
           ((SERVO_PWM_ABS_MAX - SERVO_PWM_ABS_MIN) * percent) / 100;
}

// Ativa o canal LEDC de volta após ter sido parado com ledc_stop()
static void ledc_restart_channel(ledc_channel_t channel, int gpio_num) {
    ledc_channel_config_t ch = {
        .speed_mode = LEDC_MODE,
        .channel    = channel,
        .timer_sel  = LEDC_TIMER_SERVO,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = gpio_num,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ch);
}


void servos_init(void) {
    ledc_timer_config_t timer_servo = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER_SERVO,
        .duty_resolution = LEDC_RES_SERVO,
        .freq_hz         = LEDC_FREQ_SERVO,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_servo));

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

    static float current_x_f = 0.0f;
    static float current_y_f = 0.0f;
    bool initialized = false;

    // Gate: última posição enviada ao hardware
    int last_sent_x = -1;
    int last_sent_y = -1;

    // Contadores de assentamento pós-movimento
    int settle_x = 0;
    int settle_y = 0;

    // Rastreia se o canal está ativo ou foi desligado com ledc_stop()
    bool channel_x_active = true;
    bool channel_y_active = true;

    printf(">>> servo_control_task iniciada\n");

    for (;;) {
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            local = global_data;
            xSemaphoreGive(data_mutex);
        }

        if (!initialized) {
            current_x_f = (float)percent_to_pwm(SERVO_X_START_PERCENT);
            current_y_f = (float)percent_to_pwm(SERVO_Y_START_PERCENT);
            initialized = true;
        }

        int joy_x_clean = apply_deadzone(local.joy_x_raw);
        int joy_y_clean = apply_deadzone(local.joy_y_raw);

        int target_x = map_value(joy_x_clean, ADC_MIN_READ, ADC_MAX_READ,
                                 local.servo_max_x, local.servo_min_x);
        int target_y = map_value(joy_y_clean, ADC_MIN_READ, ADC_MAX_READ,
                                 local.servo_max_y, local.servo_min_y);

        // Filtro exponencial com snap: se estiver muito perto do target,
        // força convergência exata em vez de continuar oscilando em float.
        float dx = (float)target_x - current_x_f;
        float dy = (float)target_y - current_y_f;

        if (fabsf(dx) < SERVO_SNAP_THRESHOLD) {
            current_x_f = (float)target_x;  // snap: trava no alvo
        } else {
            current_x_f += dx * SERVO_SMOOTH_FACTOR;
        }

        if (fabsf(dy) < SERVO_SNAP_THRESHOLD) {
            current_y_f = (float)target_y;
        } else {
            current_y_f += dy * SERVO_SMOOTH_FACTOR;
        }

        int new_x = (int)current_x_f;
        int new_y = (int)current_y_f;

        // ── Gate eixo X ───────────────────────────────────────────────────────
        if (abs(new_x - last_sent_x) >= SERVO_MOVE_THRESHOLD) {
            // Movimento detectado: (re)ativa canal se estava desligado
            if (!channel_x_active) {
                ledc_restart_channel(LEDC_CHANNEL_X, SERVO_X_PIN);
                channel_x_active = true;
            }
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, new_x);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X);
            last_sent_x = new_x;
            settle_x = 0;

        } else if (settle_x < SERVO_SETTLE_CYCLES) {
            // Recém parou: envia mais alguns ciclos para assentar
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, new_x);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X);
            settle_x++;

        } else if (channel_x_active) {
            // Assentou: desliga o sinal completamente para o SG90 não tremer
            ledc_stop(LEDC_MODE, LEDC_CHANNEL_X, 0);
            channel_x_active = false;
        }

        // ── Gate eixo Y (mesma lógica) ────────────────────────────────────────
        if (abs(new_y - last_sent_y) >= SERVO_MOVE_THRESHOLD) {
            if (!channel_y_active) {
                ledc_restart_channel(LEDC_CHANNEL_Y, SERVO_Y_PIN);
                channel_y_active = true;
            }
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, new_y);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y);
            last_sent_y = new_y;
            settle_y = 0;

        } else if (settle_y < SERVO_SETTLE_CYCLES) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, new_y);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y);
            settle_y++;

        } else if (channel_y_active) {
            ledc_stop(LEDC_MODE, LEDC_CHANNEL_Y, 0);
            channel_y_active = false;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}