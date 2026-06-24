#include "mpu6050.h"
#include "system_common.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// ────────────────────────────────────────────────────────────
//  Constantes internas
// ────────────────────────────────────────────────────────────

// Período da task (ms) — 10 ms = 100 Hz de amostragem
#define MPU_TASK_PERIOD_MS 10
#define MPU_DT (MPU_TASK_PERIOD_MS / 1000.0f)

// Filtro complementar: quanto do giroscópio preservar a cada ciclo.
// Alpha próximo de 1 → confia mais no giroscópio (menos ruído, mais drift).
// Alpha próximo de 0 → confia mais no acelerômetro (sem drift, mais ruído).
// 0.96 é um valor clássico para 100 Hz.
#define COMP_FILTER_ALPHA 0.96f

// Número de amostras para calibração do offset do giroscópio
#define GYRO_CALIB_SAMPLES 200

// Timeout I²C — valor generoso para o Wokwi (simulador é mais lento que hardware real)
#define I2C_TIMEOUT_MS 100

// Taxa de envio pela UART (a cada N ciclos de task)
// 10 ms × 10 = 100 ms → 10 Hz no serial (não polui o monitor)
#define UART_SEND_EVERY_N 50

// ────────────────────────────────────────────────────────────
//  Offsets de calibração (preenchidos em mpu6050_init)
// ────────────────────────────────────────────────────────────
static float gyro_offset_x = 0.0f;
static float gyro_offset_y = 0.0f;
static float gyro_offset_z = 0.0f;

// ────────────────────────────────────────────────────────────
//  Helpers I²C de baixo nível
// ────────────────────────────────────────────────────────────

static esp_err_t i2c_write_byte(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(
        MPU_I2C_PORT, MPU6050_ADDR,
        buf, sizeof(buf),
        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t i2c_read_bytes(uint8_t reg, uint8_t *dst, size_t len)
{
    return i2c_master_write_read_device(
        MPU_I2C_PORT, MPU6050_ADDR,
        &reg, 1,
        dst, len,
        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

// Lê um par de registradores (High + Low) e devolve int16
static inline int16_t read_word(const uint8_t *buf, int offset)
{
    return (int16_t)((buf[offset] << 8) | buf[offset + 1]);
}

// ────────────────────────────────────────────────────────────
//  Leitura bruta do sensor
// ────────────────────────────────────────────────────────────
typedef struct
{
    float ax, ay, az; // aceleração em g
    float gx, gy, gz; // velocidade angular em °/s
} mpu_data_t;

static bool mpu_read_raw(mpu_data_t *out)
{
    uint8_t buf[14];

    // Lê acelerômetro (6 bytes) + temperatura (2, descartados) + giroscópio (6)
    // a partir do registrador ACCEL_XOUT_H (0x3B) em rajada de 14 bytes
    if (i2c_read_bytes(MPU_REG_ACCEL_XOUT_H, buf, 14) != ESP_OK)
        return false;

    int16_t raw_ax = read_word(buf, 0);
    int16_t raw_ay = read_word(buf, 2);
    int16_t raw_az = read_word(buf, 4);
    // buf[6..7] = temperatura (ignorada aqui)
    int16_t raw_gx = read_word(buf, 8);
    int16_t raw_gy = read_word(buf, 10);
    int16_t raw_gz = read_word(buf, 12);

    out->ax = (float)raw_ax / ACCEL_SENSITIVITY;
    out->ay = (float)raw_ay / ACCEL_SENSITIVITY;
    out->az = (float)raw_az / ACCEL_SENSITIVITY;

    // subtrai offsets de calibração
    out->gx = (float)raw_gx / GYRO_SENSITIVITY - gyro_offset_x;
    out->gy = (float)raw_gy / GYRO_SENSITIVITY - gyro_offset_y;
    out->gz = (float)raw_gz / GYRO_SENSITIVITY - gyro_offset_z;

    return true;
}

// ────────────────────────────────────────────────────────────
//  Calibração estática do giroscópio
// ────────────────────────────────────────────────────────────
static void calibrate_gyro(void)
{
    printf("[MPU6050] Calibrando giroscopio (%d amostras)...\n", GYRO_CALIB_SAMPLES);

    // Pausa única para o sensor estabilizar antes de coletar
    vTaskDelay(pdMS_TO_TICKS(50));

    double sx = 0, sy = 0, sz = 0;
    int count = 0;
    uint8_t buf[6];

    // Leituras em rajada sem delay entre elas — sem risco de travar o watchdog
    for (int i = 0; i < GYRO_CALIB_SAMPLES; i++)
    {
        if (i2c_read_bytes(MPU_REG_GYRO_XOUT_H, buf, 6) == ESP_OK)
        {
            sx += (int16_t)((buf[0] << 8) | buf[1]);
            sy += (int16_t)((buf[2] << 8) | buf[3]);
            sz += (int16_t)((buf[4] << 8) | buf[5]);
            count++;
        }
        // Cede o processador a cada 20 amostras para não monopolizar a CPU
        if (i % 20 == 19)
            vTaskDelay(1);
    }

    if (count == 0)
        count = 1; // proteção contra divisão por zero
    gyro_offset_x = (float)(sx / count) / GYRO_SENSITIVITY;
    gyro_offset_y = (float)(sy / count) / GYRO_SENSITIVITY;
    gyro_offset_z = (float)(sz / count) / GYRO_SENSITIVITY;

    printf("[MPU6050] Offsets: gx=%.3f  gy=%.3f  gz=%.3f (deg/s)\n",
           gyro_offset_x, gyro_offset_y, gyro_offset_z);
}

// ────────────────────────────────────────────────────────────
//  API pública: inicialização
// ────────────────────────────────────────────────────────────
bool mpu6050_init(void)
{
    // Configura o barramento I²C como master
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU_SDA_PIN,
        .scl_io_num = MPU_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(MPU_I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(MPU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    // Aguarda o barramento estabilizar após inicialização
    vTaskDelay(pdMS_TO_TICKS(50));

    // Leitura descartável para "aquecer" o barramento I²C
    // (bug conhecido do driver legado no ESP32-C6: primeira leitura pode retornar lixo)
    uint8_t dummy = 0;
    i2c_read_bytes(MPU_REG_WHO_AM_I, &dummy, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Agora lê de verdade
    uint8_t who = 0;
    if (i2c_read_bytes(MPU_REG_WHO_AM_I, &who, 1) != ESP_OK || (who != 0x68 && who != 0x70))
    {
        printf("[MPU6050] ERRO: WHO_AM_I = 0x%02X (esperado 0x68 ou 0x70)\n", who);
        return false;
    }
    printf("[MPU6050] Sensor detectado (WHO_AM_I=0x%02X)\n", who);

    // Tira o chip do modo sleep e escolhe o clock interno do giroscópio (mais estável)
    ESP_ERROR_CHECK(i2c_write_byte(MPU_REG_PWR_MGMT_1, 0x01));

    // Sample Rate = Gyro Output / (1 + SMPLRT_DIV)
    // 1 kHz / (1 + 9) = 100 Hz
    ESP_ERROR_CHECK(i2c_write_byte(MPU_REG_SMPLRT_DIV, 0x09));

    // DLPF: filtro digital passa-baixas em ~44 Hz (reduz ruído de alta frequência)
    ESP_ERROR_CHECK(i2c_write_byte(MPU_REG_CONFIG, 0x03));

    // Escala do giroscópio: ±250 °/s (00 nos bits [4:3])
    ESP_ERROR_CHECK(i2c_write_byte(MPU_REG_GYRO_CONFIG, 0x00));

    // Escala do acelerômetro: ±2 g (00 nos bits [4:3])
    ESP_ERROR_CHECK(i2c_write_byte(MPU_REG_ACCEL_CONFIG, 0x00));

    vTaskDelay(pdMS_TO_TICKS(100)); // aguarda estabilização

    // Calibração é feita dentro da mpu6050_task para não bloquear app_main
    return true;
}

// ────────────────────────────────────────────────────────────
//  Task FreeRTOS do MPU6050
// ────────────────────────────────────────────────────────────
void mpu6050_task(void *pvParameters)
{
    printf(">>> mpu6050_task iniciada\n");

    // Calibração aqui: roda no contexto da task dedicada,
    // não bloqueia app_main nem aciona o Task Watchdog
    calibrate_gyro();

    mpu_data_t sensor;
    float pitch = 0.0f;
    float roll = 0.0f;
    bool initialized = false;
    int uart_counter = 0;

    for (;;)
    {
        if (!mpu_read_raw(&sensor))
        {
            // Falha de leitura: aguarda e tenta de novo sem atualizar os ângulos
            vTaskDelay(pdMS_TO_TICKS(MPU_TASK_PERIOD_MS));
            continue;
        }

        // ── Ângulos só pelo acelerômetro (referência absoluta, sem drift) ──
        float accel_pitch = atan2f(sensor.ay,
                                   sqrtf(sensor.ax * sensor.ax + sensor.az * sensor.az)) *
                            (float)RAD_TO_DEG;
        float accel_roll = atan2f(-sensor.ax,
                                  sqrtf(sensor.ay * sensor.ay + sensor.az * sensor.az)) *
                           (float)RAD_TO_DEG;

        // Na primeira iteração usa só o acelerômetro para inicializar
        if (!initialized)
        {
            pitch = accel_pitch;
            roll = accel_roll;
            initialized = true;
        }
        else
        {
            // ── Filtro complementar ──────────────────────────────────────────
            // Combina integração do giroscópio (rápida, sem drift de curto prazo)
            // com o acelerômetro (referência de longo prazo, mais ruidoso).
            pitch = COMP_FILTER_ALPHA * (pitch + sensor.gx * MPU_DT) + (1.0f - COMP_FILTER_ALPHA) * accel_pitch;
            roll = COMP_FILTER_ALPHA * (roll + sensor.gy * MPU_DT) + (1.0f - COMP_FILTER_ALPHA) * accel_roll;
        }

        // ── Grava na estrutura global (thread-safe) ──────────────────────────
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE)
        {
            global_data.filtered_pitch = pitch;
            global_data.filtered_roll = roll;

            // Se o botão pediu re-calibração, recoloca os ângulos na referência
            // do acelerômetro e limpa a flag.
            if (global_data.calibration_trigger)
            {
                pitch = accel_pitch;
                roll = accel_roll;
                global_data.calibration_trigger = false;
                printf("[MPU6050] Recalibracao de angulos executada\n");
            }

            xSemaphoreGive(data_mutex);
        }

        // ── Envio periódico via UART (JSON) ──────────────────────────────────
        uart_counter++;
        if (uart_counter >= UART_SEND_EVERY_N)
        {
            uart_counter = 0;

            // Captura joystick e botão para incluir no JSON
            int jx, jy;
            bool btn;
            if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE)
            {
                jx = global_data.joy_x_raw;
                jy = global_data.joy_y_raw;
                btn = global_data.btn_pressed;
                xSemaphoreGive(data_mutex);
            }
            else
            {
                jx = jy = 0;
                btn = false;
            }

            // Formato JSON compatível com qualquer parser no PC
            // Exemplo de saída:
            // {"pitch":-1.23,"roll":4.56,"ax":0.01,"ay":0.03,"az":0.99,
            //  "gx":0.00,"gy":0.01,"gz":0.00,"joy_x":2048,"joy_y":2048,"btn":0}
            printf("{\"pitch\":%.2f,\"roll\":%.2f,"
                   "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
                   "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
                   "\"joy_x\":%d,\"joy_y\":%d,\"btn\":%d}\n",
                   pitch, roll,
                   sensor.ax, sensor.ay, sensor.az,
                   sensor.gx, sensor.gy, sensor.gz,
                   jx, jy, btn ? 1 : 0);
        }

        vTaskDelay(pdMS_TO_TICKS(MPU_TASK_PERIOD_MS));
    }
}