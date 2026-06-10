#ifndef SYSTEM_COMMON_H
#define SYSTEM_COMMON_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define RAD_TO_DEG 57.2957795131

// estrutura compartilhada entre as tasks
typedef struct {
    int joy_x_raw;
    int joy_y_raw;
    bool btn_pressed;

    // ângulos do MPU6050 (Fase 2)
    float filtered_pitch;
    float filtered_roll;

    // flags de calibração (Fase 2)
    bool calibration_mode;
    bool calibration_trigger;

    // limites de PWM dos servos
    int servo_min_x;
    int servo_max_x;
    int servo_min_y;
    int servo_max_y;

    // flag de vitória (extra)
    bool game_won;

    // sinaliza fim da inicialização (usado pela task de status)
    bool system_ready;
} system_data_t;

extern system_data_t global_data;
extern SemaphoreHandle_t data_mutex;

#endif