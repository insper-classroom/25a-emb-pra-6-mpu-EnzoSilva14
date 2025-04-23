#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include "Fusion.h"

#define SAMPLE_INTERVAL_SEC (0.01f)    // Período de amostragem em segundos

static const int MPU_ADDR       = 0x68;
static const int PIN_I2C_SDA    = 4;
static const int PIN_I2C_SCL    = 5;

typedef struct {
    uint8_t axis_id;   // 0 = Roll, 1 = Pitch, 2 = Aceleração X
    int16_t value;     // Valor convertido
} IMUMessage_t;

static QueueHandle_t imuQueue;

// Reinicia o sensor para o modo operacional
static void reset_mpu6050(void) {
    uint8_t cmd[2] = { 0x6B, 0x00 };
    i2c_write_blocking(i2c_default, MPU_ADDR, cmd, 2, false);
}

// Lê 3 eixos de aceleração, giroscópio e temperatura bruta
static void read_mpu6050_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buf[6];
    uint8_t reg;

    // Leitura de aceleração (0x3B…0x40)
    reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDR, buf, 6, false);
    for (int i = 0; i < 3; i++)
        accel[i] = (buf[2*i] << 8) | buf[2*i+1];

    // Leitura de giroscópio (0x43…0x48)
    reg = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDR, buf, 6, false);
    for (int i = 0; i < 3; i++)
        gyro[i] = (buf[2*i] << 8) | buf[2*i+1];

    // Leitura de temperatura (0x41…0x42)
    reg = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDR, buf, 2, false);
    *temp = (buf[0] << 8) | buf[1];
}

// Tarefa que faz leitura do MPU, fusão de dados e empacotamento para fila
void vTaskMPU(void *pvParameters) {
    // Inicializa I2C a 400 kHz
    i2c_init(i2c_default, 400000);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    FusionAhrs fusion;
    FusionAhrsInitialise(&fusion);
    reset_mpu6050();

    int16_t accel_raw[3], gyro_raw[3], temp_raw;

    for (;;) {
        read_mpu6050_raw(accel_raw, gyro_raw, &temp_raw);

        FusionVector gyroVec = {
            .axis.x = gyro_raw[0] / 131.0f,
            .axis.y = gyro_raw[1] / 131.0f,
            .axis.z = gyro_raw[2] / 131.0f,
        };
        FusionVector accelVec = {
            .axis.x = accel_raw[0] / 16384.0f,
            .axis.y = accel_raw[1] / 16384.0f,
            .axis.z = accel_raw[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&fusion, gyroVec, accelVec, SAMPLE_INTERVAL_SEC);
        FusionEuler angles = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&fusion));

        // Enviar Roll e Pitch
        IMUMessage_t msg;
        msg.axis_id = 0;
        msg.value   = (int16_t)angles.angle.roll;
        xQueueSend(imuQueue, &msg, 0);

        msg.axis_id = 1;
        msg.value   = (int16_t)angles.angle.pitch;
        xQueueSend(imuQueue, &msg, 0);

        // Se aceleração repentina no X ultrapassar threshold de 1.5g
        int16_t accelXmg = (int16_t)(accelVec.axis.x * 100);
        if (abs(accelXmg) > 100) {
            msg.axis_id = 2;
            msg.value   = accelXmg;
            xQueueSend(imuQueue, &msg, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Tarefa que consome da fila e transmite via UART em pacotes de 4 bytes
void vTaskUART(void *pvParameters) {
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    IMUMessage_t received;
    uint8_t packet[4];

    for (;;) {
        if (xQueueReceive(imuQueue, &received, pdMS_TO_TICKS(100))) {
            packet[0] = 0xFF;
            packet[1] = received.axis_id;
            packet[2] = (uint8_t)(received.value & 0xFF);
            packet[3] = (uint8_t)((received.value >> 8) & 0xFF);
            uart_write_blocking(uart0, packet, sizeof(packet));
        }
    }
}

int main(void) {
    stdio_init_all();
    imuQueue = xQueueCreate(32, sizeof(IMUMessage_t));

    xTaskCreate(vTaskMPU,  "MPU_Task",  8192, NULL, 1, NULL);
    xTaskCreate(vTaskUART, "UART_Task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();
    for (;;);
}
