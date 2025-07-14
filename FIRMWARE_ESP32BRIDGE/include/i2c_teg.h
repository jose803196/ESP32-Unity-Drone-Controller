#include "driver/i2c_master.h"

#define I2C_MASTER_SCL_IO           22          // Pin SCL
#define I2C_MASTER_SDA_IO           21          // Pin SDA
#define I2C_PORT_NUM_0              I2C_NUM_0   // Puerto I2C
#define I2C_MASTER_FREQ_HZ          100000      // Frecuencia estándar
#define I2C_MASTER_TX_BUF_DISABLE   0           // Sin buffer de transmisión
#define I2C_MASTER_RX_BUF_DISABLE   0           // Sin buffer de recepción
#define I2C_MASTER_TIMEOUT_MS       1000

esp_err_t i2c_master_init(void);
i2c_master_bus_handle_t get_i2c_bus_handle(void);