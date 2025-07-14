#include <stdio.h>
#include "esp_err.h"
#include "i2c_teg.h"

#define MPU6050_ADDR         0x68   // Dirección I2C del MPU6050
#define MPU6050_WHO_AM_I     0x75   // Registro de identificación
#define MPU6050_PWR_MGMT_1   0x6B   // Registro de gestión de energía

//Ratios de conversion
#define A_R 16384.0     //32768/2
#define G_R 131.0       // 32768/250

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} mpu6050_data_t;

esp_err_t mpu6050_write_byte(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t data);
esp_err_t mpu6050_read_byte(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t *data);
esp_err_t mpu6050_read_bytes(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t *data, size_t len);
void mpu6050_init(i2c_master_dev_handle_t i2c_dev);
int16_t combine_bytes(uint8_t msb, uint8_t lsb);
void mpu6050_read_accel_gyro(i2c_master_dev_handle_t i2c_dev, mpu6050_data_t *data);