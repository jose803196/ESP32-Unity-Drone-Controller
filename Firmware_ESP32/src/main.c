#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "i2c_teg.h"
#include "mpu6050.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

// --- Pines de conexión ---
#define BOTON_AVANZAR    GPIO_NUM_32
#define BOTON_RETROCEDER GPIO_NUM_33
#define BOTON_IZQUIERDA  GPIO_NUM_25
#define BOTON_DERECHA    GPIO_NUM_26
#define JOYSTICK_X_PIN   ADC1_CHANNEL_6
#define JOYSTICK_Y_PIN   ADC1_CHANNEL_7
#define JOYSTICK_SW_PIN  GPIO_NUM_27
#define MOTOR_PIN        GPIO_NUM_18
#define BUZZER_PIN       GPIO_NUM_19

// --- Configuración UART ---
#define UART_PORT        UART_NUM_0
#define BUF_SIZE         1024

// --- Configuración Bluetooth ---
#define SPP_SERVER_NAME "DRONE_CONTROLLER_SPP"
#define ESP_DEVICE_NAME "DroneLink_Controller"

// --- Variables globales Bluetooth ---
uint32_t spp_handle = 0;

// --- Prototipos de funciones Bluetooth ---
void configurar_bluetooth();
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void enviar_por_bluetooth(const char* datos);

// --- Tiempos de feedback ---
#define MOTOR_DURATION_MS   500  // 0.5 segundos de vibración
#define BUZZER_DURATION_MS  300  // 0.3 segundos de sonido
#define FEEDBACK_DELAY_MS   100  // Delay entre activaciones

// --- Estados del juego ---
typedef enum { RUNNING, CRASHED } GameState;

// --- Estructura para datos de sensores ---
typedef struct {
    int btn_avanzar, btn_retroceder, btn_izquierda, btn_derecha, btn_joystick;
    int joy_x, joy_y;
    mpu6050_data_t mpu_data;
} SensorData;

// --- Variables globales ---
QueueHandle_t sensor_data_queue;
GameState current_state = RUNNING;
i2c_master_dev_handle_t mpu6050_dev_handle = NULL;
TaskHandle_t motor_task_handle = NULL;
TaskHandle_t buzzer_task_handle = NULL;

// --- Prototipos de funciones ---
void configurar_gpios();
void configurar_joystick_adc();
void configurar_mpu6050();
void configurar_uart();
void enviar_por_uart(const char* datos);

// --- Tareas ---
void sensor_task(void *pvParameters);
void uart_task(void *pvParameters);
void motor_task(void *pvParameters);
void buzzer_task(void *pvParameters);

// --- Modificación de app_main ---
void app_main(void) {
    // Inicialización de NVS (necesario para Bluetooth)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicialización de hardware
    configurar_gpios();
    configurar_joystick_adc();
    configurar_mpu6050();
    configurar_uart();
    configurar_bluetooth();  // Nueva función de configuración
    
    // Crear cola para datos de sensores
    sensor_data_queue = xQueueCreate(5, sizeof(SensorData));
    
    if (sensor_data_queue != NULL) {
        // Crear tareas
        xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 10, NULL);
        xTaskCreate(uart_task, "uart_task", 4096, NULL, 9, NULL); // Para depuración
        xTaskCreate(motor_task, "motor_task", 2048, NULL, 5, &motor_task_handle);
        xTaskCreate(buzzer_task, "buzzer_task", 2048, NULL, 5, &buzzer_task_handle);
        
        enviar_por_uart("Sistema de control iniciado.\n");
    } else {
        printf("Error al crear la cola de datos\n");
    }
}

// --- Implementación de tareas ---

// --- Nueva función para configurar Bluetooth ---
void configurar_bluetooth() {
    // Liberar memoria si no se usa BLE
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    
    // Configurar controlador Bluetooth
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    
    // Inicializar Bluedroid
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    
    // Registrar callback y inicializar SPP
    ESP_ERROR_CHECK(esp_spp_register_callback(esp_spp_cb));
    ESP_ERROR_CHECK(esp_spp_init(ESP_SPP_MODE_CB));
}

// --- Callback de Bluetooth SPP ---
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            printf("SPP inicializado.\n");
            esp_bt_dev_set_device_name(ESP_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            printf("Cliente conectado. Handle: %d\n", (int)param->srv_open.handle);
            spp_handle = param->srv_open.handle;
            current_state = RUNNING; // Reiniciar estado al conectar
            break;
        case ESP_SPP_DATA_IND_EVT: // Evento de datos recibidos
            printf("Datos recibidos: %.*s\n", param->data_ind.len, (char *)param->data_ind.data);
            if (param->data_ind.len > 0) {
                if (param->data_ind.data[0] == 'C' && current_state == RUNNING) {
                    printf("Comando CRASH recibido por Bluetooth!\n");
                    current_state = CRASHED;
                    xTaskNotifyGive(motor_task_handle);
                    xTaskNotifyGive(buzzer_task_handle);
                } else if (param->data_ind.data[0] == 'R') {
                    printf("Comando RESTART recibido por Bluetooth!\n");
                    current_state = RUNNING;
                }
            }
            break;
        case ESP_SPP_CLOSE_EVT:
            printf("Cliente desconectado.\n");
            spp_handle = 0; // Resetear el handle
            break;
        default:
            break;
    }
}

// --- Función para enviar datos por Bluetooth ---
void enviar_por_bluetooth(const char* datos) {
    if (spp_handle != 0) {
        esp_spp_write(spp_handle, strlen(datos), (uint8_t *)datos);
    }
}

void sensor_task(void *pvParameters) {
    SensorData current_data;
    
    while (1) {
        // Leer todos los sensores
        current_data.btn_avanzar = !gpio_get_level(BOTON_AVANZAR);
        current_data.btn_retroceder = !gpio_get_level(BOTON_RETROCEDER);
        current_data.btn_izquierda = !gpio_get_level(BOTON_IZQUIERDA);
        current_data.btn_derecha = !gpio_get_level(BOTON_DERECHA);
        current_data.btn_joystick = !gpio_get_level(JOYSTICK_SW_PIN);
        current_data.joy_x = adc1_get_raw(JOYSTICK_X_PIN);
        current_data.joy_y = adc1_get_raw(JOYSTICK_Y_PIN);
        mpu6050_read_accel_gyro(mpu6050_dev_handle, &current_data.mpu_data);
        
        // Enviar datos a la cola
        xQueueSend(sensor_data_queue, &current_data, 0);
        
        vTaskDelay(20 / portTICK_PERIOD_MS); // 50 Hz
    }
}

// --- Modificación de la tarea UART para manejar ambos protocolos ---
void uart_task(void *pvParameters) {
    char buffer_tx[256];
    uint8_t buffer_uart_rx[32];
    SensorData received_data;
    
    while (1) {
        // Leer comandos de UART (para depuración)
        int len = uart_read_bytes(UART_PORT, buffer_uart_rx, sizeof(buffer_uart_rx) - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            buffer_uart_rx[len] = '\0';
            
            if (buffer_uart_rx[0] == 'C' && current_state == RUNNING) {
                current_state = CRASHED;
                xTaskNotifyGive(motor_task_handle);
                xTaskNotifyGive(buzzer_task_handle);
            } else if (buffer_uart_rx[0] == 'R') {
                current_state = RUNNING;
            }
        }
        
        // Procesar datos de sensores si estamos en estado RUNNING
        if (current_state == RUNNING && xQueueReceive(sensor_data_queue, &received_data, 0) == pdTRUE) {
            snprintf(buffer_tx, sizeof(buffer_tx), 
                    "%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                    received_data.btn_avanzar, received_data.btn_retroceder, 
                    received_data.btn_izquierda, received_data.btn_derecha,
                    received_data.joy_x, received_data.joy_y, received_data.btn_joystick,
                    received_data.mpu_data.ax, received_data.mpu_data.ay, received_data.mpu_data.az,
                    received_data.mpu_data.gx, received_data.mpu_data.gy, received_data.mpu_data.gz);
            
            // Enviar por ambos canales
            enviar_por_uart(buffer_tx); // Para depuración
            enviar_por_bluetooth(buffer_tx); // Para comunicación principal
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void motor_task(void *pvParameters) {
    while (1) {
        // Esperar notificación
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // Activar motor por el tiempo configurado
        gpio_set_level(MOTOR_PIN, 1);
        vTaskDelay(MOTOR_DURATION_MS / portTICK_PERIOD_MS);
        gpio_set_level(MOTOR_PIN, 0);
        
        // Pequeño delay para evitar activaciones repetidas muy rápidas
        vTaskDelay(FEEDBACK_DELAY_MS / portTICK_PERIOD_MS);
    }
}

void buzzer_task(void *pvParameters) {
    while (1) {
        // Esperar notificación
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // Activar buzzer por el tiempo configurado
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(BUZZER_DURATION_MS / portTICK_PERIOD_MS);
        gpio_set_level(BUZZER_PIN, 0);
        
        // Pequeño delay para evitar activaciones repetidas muy rápidas
        vTaskDelay(FEEDBACK_DELAY_MS / portTICK_PERIOD_MS);
    }
}

// --- Funciones de configuración ---

void configurar_gpios() {
    gpio_config_t io_conf_in = {
        .pin_bit_mask = (1ULL << BOTON_AVANZAR) | (1ULL << BOTON_RETROCEDER) |
                         (1ULL << BOTON_IZQUIERDA) | (1ULL << BOTON_DERECHA) |
                         (1ULL << JOYSTICK_SW_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf_in);

    gpio_config_t io_conf_out = {
        .pin_bit_mask = (1ULL << MOTOR_PIN) | (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&io_conf_out);
}

void configurar_joystick_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(JOYSTICK_X_PIN, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(JOYSTICK_Y_PIN, ADC_ATTEN_DB_12);
}

void configurar_mpu6050() {
    ESP_ERROR_CHECK(i2c_master_init());
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = 100000,
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
    uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void enviar_por_uart(const char* datos) {
    uart_write_bytes(UART_PORT, datos, strlen(datos));
}