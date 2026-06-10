#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include <stdbool.h>

// ────────────────────────────────────────────────────────────
//  Pinos I²C (ajuste conforme seu diagram.json)
// ────────────────────────────────────────────────────────────
#define MPU_SDA_PIN     8
#define MPU_SCL_PIN     9
#define MPU_I2C_PORT    I2C_NUM_0
#define MPU_I2C_FREQ_HZ 400000   // Fast-mode (400 kHz)

// Endereço I²C do MPU6050 (AD0 = GND → 0x68)
#define MPU6050_ADDR    0x68

// ────────────────────────────────────────────────────────────
//  Registradores do MPU6050
// ────────────────────────────────────────────────────────────
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B   // 6 bytes: AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L
#define MPU_REG_GYRO_XOUT_H  0x43   // 6 bytes: GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L
#define MPU_REG_WHO_AM_I     0x75

// Escalas padrão
// Acelerômetro ±2 g  → 16384 LSB/g
// Giroscópio   ±250°/s → 131 LSB/(°/s)
#define ACCEL_SENSITIVITY    16384.0f
#define GYRO_SENSITIVITY     131.0f

// ────────────────────────────────────────────────────────────
//  API pública
// ────────────────────────────────────────────────────────────

/**
 * @brief Inicializa o barramento I²C e configura o MPU6050.
 *        Deve ser chamada antes de criar a task.
 * @return true se o sensor foi detectado e configurado com sucesso.
 */
bool mpu6050_init(void);

/**
 * @brief Task FreeRTOS dedicada ao MPU6050.
 *        Lê o sensor, calcula pitch/roll com filtro complementar,
 *        grava em global_data e envia JSON pela UART.
 */
void mpu6050_task(void *pvParameters);

#endif // MPU6050_DRIVER_H