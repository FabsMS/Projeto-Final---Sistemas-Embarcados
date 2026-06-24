#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "joystick.h"
#include "servo.h"
#include "status.h"
#include "system_common.h"
#include "mpu6050.h"   // Fase 2


/**
 * @brief Tarefa de Monitoramento e Telemetria.
 * O sistema utiliza esta tarefa para coletar dados de todos os módulos e enviá-los via Serial (UART).
 * Isso permite que ferramentas externas (como Grafana ou Serial Plotter) visualizem o estado do jogo em tempo real.
 */
void status_monitor_task(void *pvParameters) {
    system_data_t local;
    printf("Monitor Iniciado.\n");
    for (;;) {
        // O sistema adquire o Mutex para garantir que não lerá dados corrompidos enquanto outra tarefa está escrevendo.
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            local = global_data; // O sistema cria uma cópia local segura dos dados globais.
            xSemaphoreGive(data_mutex);
        }

        // O sistema formata os dados como uma string JSON.
        // Isso padroniza a saída para: Joystick (Input), MPU (Sensor) e Vitória (Estado do Jogo).
        printf(
            "{\"joy\":{\"x\":%d,\"y\":%d},\"mpu\":{\"ang_x\":%.2f,\"ang_y\":%.2f},\"win\":%d}\n",
            local.joy_x_raw, local.joy_y_raw,
            local.filtered_roll, local.filtered_pitch,
            local.game_won
        );

        // O sistema aguarda 200ms, definindo uma taxa de atualização de telemetria de 5Hz.
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}


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
    xTaskCreate(joystick_read_task,   "JoyRead", 2048, NULL, 3, NULL);
    xTaskCreate(servo_control_task,   "Servos",  2048, NULL, 2, NULL);
    xTaskCreate(status_task,          "Status",  2048, NULL, 1, NULL);

    // ── Telemetria Serial → Python → InfluxDB → Grafana ─────────────────────
    // Prioridade 1: mesma da status_task — só reporta, não controla nada.
    // Stack 2048: printf com floats precisa de um pouco mais que o básico.
    xTaskCreate(status_monitor_task,  "Monitor", 2048, NULL, 1, NULL);

    printf("[MAIN] Sistema iniciado.\n");
}