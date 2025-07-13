#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "i2c_teg.h"
#include "mpu6050.h"

// --- PINES DE CONEXIÓN ---
// Entradas
#define BOTON_AVANZAR    GPIO_NUM_32
#define BOTON_RETROCEDER GPIO_NUM_33
#define BOTON_IZQUIERDA  GPIO_NUM_25
#define BOTON_DERECHA    GPIO_NUM_26
#define JOYSTICK_X_PIN   ADC1_CHANNEL_6 // GPIO 34
#define JOYSTICK_Y_PIN   ADC1_CHANNEL_7 // GPIO 35
#define JOYSTICK_SW_PIN  GPIO_NUM_27
// Salidas para feedback
#define MOTOR_PIN        GPIO_NUM_18
#define BUZZER_PIN       GPIO_NUM_19

// --- CONFIGURACIÓN UART ---
#define UART_PORT        UART_NUM_0
#define BUF_SIZE         1024

// --- LÓGICA DE ESTADO DEL JUEGO ---
typedef enum {
    RUNNING,
    CRASHED
} GameState;

GameState current_state = RUNNING; // Empezamos en estado normal

// --- VARIABLES GLOBALES ---
i2c_master_dev_handle_t mpu6050_dev_handle = NULL;
mpu6050_data_t mpu_data;

// --- FUNCIONES DE CONFIGURACIÓN ---

void configurar_gpios() {
    // Configuración de pines de ENTRADA (botones)
    gpio_config_t io_conf_in = {
        .pin_bit_mask = (1ULL << BOTON_AVANZAR) | (1ULL << BOTON_RETROCEDER) |
                         (1ULL << BOTON_IZQUIERDA) | (1ULL << BOTON_DERECHA) |
                         (1ULL << JOYSTICK_SW_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf_in);

    // Configuración de pines de SALIDA (motor y buzzer)
    gpio_config_t io_conf_out = {
        .pin_bit_mask = (1ULL << MOTOR_PIN) | (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&io_conf_out);
}

void configurar_joystick_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(JOYSTICK_X_PIN, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(JOYSTICK_Y_PIN, ADC_ATTEN_DB_11);
}

void configurar_mpu6050_con_tus_drivers() {
    ESP_ERROR_CHECK(i2c_master_init());
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(get_i2c_bus_handle(), &dev_cfg, &mpu6050_dev_handle));
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
    uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0); // Habilitamos buffer de RX
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void enviar_por_uart(const char* datos) {
    uart_write_bytes(UART_PORT, datos, strlen(datos));
}

// --- FUNCIONES DE FEEDBACK ---
void activar_feedback_choque() {
    gpio_set_level(MOTOR_PIN, 1);
    gpio_set_level(BUZZER_PIN, 1);
}

void desactivar_feedback_choque() {
    gpio_set_level(MOTOR_PIN, 0);
    gpio_set_level(BUZZER_PIN, 0);
}

// --- FUNCIÓN PRINCIPAL ---
void app_main(void) {
    // Inicializar hardware
    configurar_gpios(); // Función actualizada
    configurar_joystick_adc();
    configurar_mpu6050_con_tus_drivers();
    configurar_uart();
    
    // Asegurarse de que el feedback esté apagado al inicio
    desactivar_feedback_choque();
    
    enviar_por_uart("Sistema de control iniciado.\n");

    char buffer_uart_tx[256]; // Buffer para enviar datos
    uint8_t buffer_uart_rx[32]; // Buffer para recibir comandos

    while (1) {
        // --- 1. LEER COMANDOS DE UNITY ---
        int len = uart_read_bytes(UART_PORT, buffer_uart_rx, sizeof(buffer_uart_rx) - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            buffer_uart_rx[len] = '\0'; // Terminador nulo
            if (buffer_uart_rx[0] == 'C') {
                current_state = CRASHED;
                activar_feedback_choque();
            } else if (buffer_uart_rx[0] == 'R') {
                current_state = RUNNING;
                desactivar_feedback_choque();
            }
        }

        // --- 2. EJECUTAR LÓGICA BASADA EN EL ESTADO ---
        if (current_state == RUNNING) {
            // Si estamos en estado RUNNING, leemos sensores y enviamos datos.
            
            // Leer botones
            int avanzar = !gpio_get_level(BOTON_AVANZAR);
            int retroceder = !gpio_get_level(BOTON_RETROCEDER);
            int izquierda = !gpio_get_level(BOTON_IZQUIERDA);
            int derecha = !gpio_get_level(BOTON_DERECHA);
            int joystick_sw = !gpio_get_level(JOYSTICK_SW_PIN);

            // Leer Joystick
            int joystick_x = adc1_get_raw(JOYSTICK_X_PIN);
            int joystick_y = adc1_get_raw(JOYSTICK_Y_PIN);
            
            // Leer MPU6050
            mpu6050_read_accel_gyro(mpu6050_dev_handle, &mpu_data);

            // Formatear y enviar datos a Unity
            snprintf(buffer_uart_tx, sizeof(buffer_uart_tx), 
                    "%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                    avanzar, retroceder, izquierda, derecha,
                    joystick_x, joystick_y, joystick_sw,
                    mpu_data.ax, mpu_data.ay, mpu_data.az,
                    mpu_data.gx, mpu_data.gy, mpu_data.gz
            );
            enviar_por_uart(buffer_uart_tx);
        
        } else {
            // Si estamos en estado CRASHED, no hacemos nada más que esperar.
            // El feedback ya está activo. El bucle seguirá leyendo el UART
            // esperando el comando 'R'.
        }
        
        // El delay se mantiene fuera del if para que el bucle siga corriendo
        // y pueda leer comandos de Unity en cualquier estado.
        vTaskDelay(50 / portTICK_PERIOD_MS);  // 20 Hz
    }
}