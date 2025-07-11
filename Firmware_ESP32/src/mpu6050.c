#include "mpu6050.h"
#include "i2c_teg.h"
#include "freertos/FreeRTOS.h"

esp_err_t mpu6050_write_byte(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t data){
    return i2c_master_transmit(i2c_dev,
        (uint8_t[]){reg_addr, data}, 2,
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_read_byte(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t *data){
    i2c_master_transmit(i2c_dev, &reg_addr, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return i2c_master_receive(i2c_dev, data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_read_bytes(i2c_master_dev_handle_t i2c_dev, uint8_t reg_addr, uint8_t *data, size_t len){
    i2c_master_transmit(i2c_dev, &reg_addr, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return i2c_master_receive(i2c_dev, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

int16_t combine_bytes(uint8_t msb, uint8_t lsb){
    return (int16_t)((msb << 8) | lsb);
}

void mpu6050_read_accel_gyro(i2c_master_dev_handle_t i2c_dev, mpu6050_data_t *data) {
    uint8_t raw_data[14]; // 6 bytes acelerómetro + 2 temp + 6 bytes giroscopio

    // Leer los 14 bytes de datos de una vez (eficiente)
    esp_err_t ret = mpu6050_read_bytes(i2c_dev, 0x3B, raw_data, 14);
    if (ret != ESP_OK) {
        printf("Error leyendo datos del MPU6050: %s\n", esp_err_to_name(ret));
        // Opcional: poner los datos a cero en caso de error
        data->ax = data->ay = data->az = 0;
        data->gx = data->gy = data->gz = 0;
        return;
    }
    data->ax = combine_bytes(raw_data[0], raw_data[1]) / A_R;
    data->ay = combine_bytes(raw_data[2], raw_data[3]) / A_R;
    data->az = combine_bytes(raw_data[4], raw_data[5]) / A_R;

    data->gx = combine_bytes(raw_data[8], raw_data[9]) / G_R;
    data->gy = combine_bytes(raw_data[10], raw_data[11]) / G_R;
    data->gz = combine_bytes(raw_data[12], raw_data[13]) / G_R;
}

void mpu6050_init(i2c_master_dev_handle_t i2c_dev){
    esp_err_t ret;

    // Despertar el MPU6050 escribiendo 0 en PWR_MGMT_1
    ret = mpu6050_write_byte(i2c_dev, MPU6050_PWR_MGMT_1, 0x00);
    if (ret == ESP_OK) {
        printf("MPU6050 inicializado correctamente\n");
    } else {
        printf("Error inicializando el MPU6050: %s\n", esp_err_to_name(ret));
    }

    // Leer WHO_AM_I
    uint8_t who_am_i = 0;
    ret = mpu6050_read_byte(i2c_dev, MPU6050_WHO_AM_I, &who_am_i);
    if (ret == ESP_OK) {
        printf("WHO_AM_I = 0x%02X\n", who_am_i);
        if (who_am_i == MPU6050_ADDR) {
            printf("MPU6050 detectado correctamente\n");
        } else {
            printf("ID del dispositivo inesperado\n");
        }
    } else {
        printf("Error leyendo WHO_AM_I: %s\n", esp_err_to_name(ret));
    }
}

