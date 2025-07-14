#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "mpu6050.h"

// --- NUEVO: Estructura de datos ampliada (DEBE ser IDÉNTICA a la del controlador) ---
typedef struct {
    int btn_avanzar, btn_retroceder, btn_izquierda, btn_derecha, btn_joystick;
    int joy_x, joy_y;
    mpu6050_data_t mpu_data;
} SensorData;

#define UART_PC_PORT UART_NUM_0 
#define UART_BUF_SIZE 1024

// --- MAC del ESP32 Controlador ---
static uint8_t mac_controlador_emisor[] = {0xFC, 0xE8, 0xC0, 0x7C, 0xA3, 0xA8};

// Prototipos
void configurar_uart_pc();
void configurar_wifi_y_espnow();
static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len);
void uart_task(void *pvParameters); // --- NUEVO: Tarea para leer de Unity ---


void app_main(void) {
    nvs_flash_init();
    configurar_uart_pc();
    configurar_wifi_y_espnow();
    
    // --- NUEVO: Crear la tarea que escucha a Unity ---
    xTaskCreate(uart_task, "uart_task", 2048, NULL, 5, NULL);

    ESP_LOGI("PUENTE", "Sistema completo listo.");
}

void configurar_wifi_y_espnow() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, mac_controlador_emisor, 6);
    peer_info.channel = 0;
    peer_info.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
}

// Callback para recibir datos de los sensores
static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (len == sizeof(SensorData)) {
        SensorData received_data;
        memcpy(&received_data, data, len);
        
        // --- NUEVO: Cadena de texto formateada con todos los datos ---
        char buffer_tx[256];
        snprintf(buffer_tx, sizeof(buffer_tx), 
                "%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                received_data.btn_avanzar, received_data.btn_retroceder, 
                received_data.btn_izquierda, received_data.btn_derecha,
                received_data.btn_joystick,
                received_data.joy_x, received_data.joy_y,
                received_data.mpu_data.ax, received_data.mpu_data.ay, received_data.mpu_data.az,
                received_data.mpu_data.gx, received_data.mpu_data.gy, received_data.mpu_data.gz);
        
        uart_write_bytes(UART_PC_PORT, buffer_tx, strlen(buffer_tx));
    }
}

// Dentro de la uart_task del ESP32 Puente
void uart_task(void *pvParameters) {
    uint8_t *data = (uint8_t*) malloc(UART_BUF_SIZE);
    while(1) {
        int len = uart_read_bytes(UART_PC_PORT, data, UART_BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            // Si Unity nos envía 'C', lo reenviamos.
            if (data[0] == 'C') {
                ESP_LOGI("PUENTE", "Comando 'C' recibido. Reenviando...");
                esp_now_send(mac_controlador_emisor, data, 1);
            }
            // NUEVO: Si Unity nos envía 'R', también lo reenviamos.
            else if (data[0] == 'R') {
                ESP_LOGI("PUENTE", "Comando 'R' recibido. Reenviando...");
                esp_now_send(mac_controlador_emisor, data, 1);
            }
        }
    }
    free(data);
}

// Configuración del UART (sin cambios)
void configurar_uart_pc() {
    uart_config_t uart_config = {
        .baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PC_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PC_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PC_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}