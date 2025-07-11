#include "i2c_teg.h"

static i2c_master_bus_handle_t bus_handle = NULL;

esp_err_t i2c_master_init(void){
    i2c_master_bus_config_t i2c_mst_config = {
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_NUM_0,
        .glitch_ignore_cnt = 7,
    };
    return i2c_new_master_bus(&i2c_mst_config, &bus_handle);
}

i2c_master_bus_handle_t get_i2c_bus_handle(void){
    return bus_handle;
}