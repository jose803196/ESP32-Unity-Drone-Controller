#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "i2c_teg.h"
#include "mpu6050.h"

// Pines de conexión
#define BOTON_AVANZAR    GPIO_NUM_32
#define BOTON_RETROCEDER GPIO_NUM_33
#define BOTON_IZQUIERDA  GPIO_NUM_25
#define BOTON_DERECHA    GPIO_NUM_26
#define JOYSTICK_X_PIN   ADC1_CHANNEL_6 // GPIO 34
#define JOYSTICK_Y_PIN   ADC1_CHANNEL_7 // GPIO 35
#define JOYSTICK_SW_PIN  GPIO_NUM_27

// Configuración del UART
#define UART_PORT        UART_NUM_0
#define BUF_SIZE         1024

// Variables globales para los sensores
i2c_master_dev_handle_t mpu6050_dev_handle = NULL;  // Handle para el dispositivo MPU6050
mpu6050_data_t mpu_data;                            // Estructura para almacenar los datos del MPU6050

// Funciones de configuración

void configurar_gpio_botones() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOTON_AVANZAR) | (1ULL << BOTON_RETROCEDER) |
                         (1ULL << BOTON_IZQUIERDA) | (1ULL << BOTON_DERECHA) |
                         (1ULL << JOYSTICK_SW_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void configurar_joystick_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12); // Rango de 0 a 4095
    adc1_config_channel_atten(JOYSTICK_X_PIN, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(JOYSTICK_Y_PIN, ADC_ATTEN_DB_11);
}

void configurar_mpu6050_con_tus_drivers() {
    // 1. Inicializa el bus I2C usando tu función de i2c_teg.c
    ESP_ERROR_CHECK(i2c_master_init());
    
    // 2. Configura el dispositivo MPU6050 en el bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    // 3. Añade el dispositivo al bus y obtén el handle
    ESP_ERROR_CHECK(i2c_master_bus_add_device(get_i2c_bus_handle(), &dev_cfg, &mpu6050_dev_handle));

    // 4. Inicializa el MPU6050 usando tu función de mpu6050.c
    mpu6050_init(mpu6050_dev_handle);
}

void configurar_uart() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_PORT, BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void enviar_por_uart(const char* datos) {
    uart_write_bytes(UART_PORT, datos, strlen(datos));
}


void app_main(void) {
    // Inicializar hardware
    configurar_gpio_botones();
    configurar_joystick_adc();
    configurar_mpu6050_con_tus_drivers(); // Usamos la nueva función
    configurar_uart();
    
    enviar_por_uart("Sistema de control iniciado con drivers personalizados.\n");

    char buffer_uart[256]; // Aumentamos el buffer por los floats

    while (1) {
        // --- LECTURA DE DATOS ---

        // 1. Leer botones (1 = presionado, 0 = no presionado)
        int avanzar = !gpio_get_level(BOTON_AVANZAR);
        int retroceder = !gpio_get_level(BOTON_RETROCEDER);
        int izquierda = !gpio_get_level(BOTON_IZQUIERDA);
        int derecha = !gpio_get_level(BOTON_DERECHA);
        int joystick_sw = !gpio_get_level(JOYSTICK_SW_PIN);

        // 2. Leer Joystick
        int joystick_x = adc1_get_raw(JOYSTICK_X_PIN);
        int joystick_y = adc1_get_raw(JOYSTICK_Y_PIN);
        
        // 3. Leer MPU6050 usando tu función modificada
        mpu6050_read_accel_gyro(mpu6050_dev_handle, &mpu_data);

        // --- FORMATEO Y ENVÍO POR UART ---
        
        // Formatear el string en formato CSV
        // Usamos %.2f para enviar los floats con 2 decimales
        snprintf(buffer_uart, sizeof(buffer_uart), 
                "%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                avanzar, retroceder, izquierda, derecha,
                joystick_x, joystick_y, joystick_sw,
                mpu_data.ax, mpu_data.ay, mpu_data.az,
                mpu_data.gx, mpu_data.gy, mpu_data.gz
        );
        
        // Enviar el paquete de datos
        enviar_por_uart(buffer_uart);
        
        vTaskDelay(50 / portTICK_PERIOD_MS);  // 20 Hz
    }
}