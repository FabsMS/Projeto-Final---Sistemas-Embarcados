#include "system_common.h"

// valores iniciais do estado global
system_data_t global_data = {
    .joy_x_raw = 2048,
    .joy_y_raw = 2048,
    .btn_pressed = false,

    .filtered_pitch = 0.0f,
    .filtered_roll  = 0.0f,

    .calibration_mode    = false,
    .calibration_trigger = false,

    // limites de PWM definidos na prática (13 bits, 50Hz)
    .servo_min_x = 700,
    .servo_max_x = 900,
    .servo_min_y = 675,
    .servo_max_y = 825,

    .game_won = false,
    .system_ready = false
};

SemaphoreHandle_t data_mutex = NULL;