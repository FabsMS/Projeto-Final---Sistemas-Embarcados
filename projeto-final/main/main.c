#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "joystick.h"
#include "servo.h"
#include "status.h"
#include "system_common.h"
#include "mpu6050.h"   // Fase 2


void app_main(void)
{
    // mutex que protege global_data entre todas as tasks
    data_mutex = xSemaphoreCreateMutex();

    // ── Inicialização de hardware ────────────────────────────────────────────
    joystick_init();
    servos_init();
    status_init();

    // mpu6050_init() configura o I²C, verifica o chip e calibra o giroscópio.
    // Se falhar (sensor ausente / fiação errada) o sistema continua sem ele.
    if (!mpu6050_init()) {
        printf("[MAIN] AVISO: MPU6050 nao inicializado. Task de IMU nao sera criada.\n");
    } else {
        // Prioridade 2: abaixo do joystick (3) para não monopolizar o core único
        // no Wokwi (ESP32-C6 single-core). Stack 4096: float + printf exigem mais.
        xTaskCreate(mpu6050_task, "MPU6050", 4096, NULL, 2, NULL);
    }

    // ── Tasks da Fase 1 ──────────────────────────────────────────────────────
    xTaskCreate(joystick_read_task, "JoyRead", 2048, NULL, 3, NULL);
    xTaskCreate(servo_control_task, "Servos",  2048, NULL, 2, NULL);
    xTaskCreate(status_task,        "Status",  2048, NULL, 1, NULL);

    printf("[MAIN] Sistema iniciado.\n");
}