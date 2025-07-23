// =================================================================================
// CÓDIGO COMPLETO Y FINAL USANDO LOS PINES 12, 13 Y 14 PARA LOS LEDS
// RECUERDA: ES NECESARIO AÑADIR UNA RESISTENCIA DE PULL-DOWN DE 10kΩ EN GPIO 12.
// =================================================================================
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "i2c_teg.h" 
#include "mpu6050.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"

// --- MAPEO DE PINES AJUSTADO A TU HARDWARE (12, 13, 14) ---
#define LED_G_PIN        GPIO_NUM_12    // Verde (RUNNING) en GPIO 12 (Necesita pull-down de 10k)
#define LED_B_PIN        GPIO_NUM_13    // Azul (INICIO) en GPIO 13 (Pin seguro)
#define LED_R_PIN        GPIO_NUM_14    // Rojo (CRASH) en GPIO 14 (Pin seguro)

// Resto de pines (Mapeo "Seguro" anterior)
#define JOYSTICK_X_PIN   ADC1_CHANNEL_4 // GPIO32
#define JOYSTICK_Y_PIN   ADC1_CHANNEL_5 // GPIO33
#define JOYSTICK_SW_PIN  GPIO_NUM_27
#define BOTON_IZQUIERDA  GPIO_NUM_25
#define BOTON_DERECHA    GPIO_NUM_26
#define BOTON_AVANZAR    GPIO_NUM_17
#define BOTON_RETROCEDER GPIO_NUM_16
#define MOTOR_PIN        GPIO_NUM_18
#define BUZZER_PIN       GPIO_NUM_19
// =========================================================

#define MOTOR_DURATION_MS   500
#define BUZZER_DURATION_MS  300

static uint8_t mac_puente_receptor[] = {0x8C, 0x4F, 0x00, 0xAB, 0x9B, 0x60};
typedef enum { RUNNING, CRASHED } GameState;
GameState current_state = RUNNING;
TaskHandle_t feedback_task_handle = NULL;
i2c_master_dev_handle_t mpu6050_dev_handle = NULL;

typedef struct {
    int btn_avanzar, btn_retroceder, btn_izquierda, btn_derecha, btn_joystick;
    int joy_x, joy_y;
    mpu6050_data_t mpu_data;
} SensorData;

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len);
void configurar_gpios();
void configurar_joystick_adc();
void configurar_mpu6050(); 
void sensor_task(void *pvParameters);
void feedback_task(void *pvParameters);

void app_main(void) {
    nvs_flash_init();
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
    memcpy(peer_info.peer_addr, mac_puente_receptor, 6);
    peer_info.channel = 0;
    peer_info.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    
    configurar_gpios();
    
    // Estado inicial "Buscando": Azul encendido
    gpio_set_level(LED_B_PIN, 1);

    configurar_joystick_adc();
    configurar_mpu6050();
    
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 10, NULL);
    xTaskCreate(feedback_task, "feedback_task", 2048, NULL, 5, &feedback_task_handle);
    
    ESP_LOGI("CONTROLADOR", "Sistema robusto iniciado.");
}

void sensor_task(void *pvParameters) {
    SensorData current_data;

    while (1) {
        // En cada ciclo, esta tarea refuerza el estado del LED
        if (current_state == RUNNING) {
            gpio_set_level(LED_B_PIN, 0);
            gpio_set_level(LED_R_PIN, 0);
            gpio_set_level(LED_G_PIN, 1);
        }

        current_data.btn_avanzar = !gpio_get_level(BOTON_AVANZAR);
        current_data.btn_retroceder = !gpio_get_level(BOTON_RETROCEDER);
        current_data.btn_izquierda = !gpio_get_level(BOTON_IZQUIERDA);
        current_data.btn_derecha = !gpio_get_level(BOTON_DERECHA);
        current_data.btn_joystick = !gpio_get_level(JOYSTICK_SW_PIN);
        current_data.joy_x = adc1_get_raw(JOYSTICK_X_PIN);
        current_data.joy_y = adc1_get_raw(JOYSTICK_Y_PIN);
        mpu6050_read_accel_gyro(mpu6050_dev_handle, &current_data.mpu_data);
        
        if (current_state == RUNNING) {
            esp_now_send(mac_puente_receptor, (uint8_t *)&current_data, sizeof(current_data));
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (len > 0) {
        if (data[0] == 'C' && current_state == RUNNING) {
            current_state = CRASHED;
            xTaskNotifyGive(feedback_task_handle);
        } else if (data[0] == 'R') {
            current_state = RUNNING;
        }
    }
}

void feedback_task(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        if(current_state == CRASHED) {
            gpio_set_level(LED_G_PIN, 0);
            gpio_set_level(LED_B_PIN, 0);
            
            gpio_set_level(MOTOR_PIN, 1);
            gpio_set_level(BUZZER_PIN, 1);
            
            for(int i = 0; i < 5; i++) {
                gpio_set_level(LED_R_PIN, 1);
                vTaskDelay((MOTOR_DURATION_MS / 10) / portTICK_PERIOD_MS);
                gpio_set_level(LED_R_PIN, 0);
                vTaskDelay((MOTOR_DURATION_MS / 10) / portTICK_PERIOD_MS);
            }
            
            gpio_set_level(BUZZER_PIN, 0);
            gpio_set_level(MOTOR_PIN, 0);
            
            gpio_set_level(LED_R_PIN, 1);
        }
    }
}

void configurar_gpios() {
    gpio_config_t io_conf_in = {
        .pin_bit_mask = (1ULL << BOTON_AVANZAR) | (1ULL << BOTON_RETROCEDER) |
                         (1ULL << BOTON_IZQUIERDA) | (1ULL << BOTON_DERECHA) |
                         (1ULL << JOYSTICK_SW_PIN),
        .mode = GPIO_MODE_INPUT, 
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf_in);
    
    gpio_config_t io_conf_out = {
        .pin_bit_mask = (1ULL << MOTOR_PIN) | (1ULL << BUZZER_PIN) | 
                        (1ULL << LED_R_PIN) | (1ULL << LED_G_PIN) | (1ULL << LED_B_PIN),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&io_conf_out);
    
    gpio_set_level(MOTOR_PIN, 0);
    gpio_set_level(BUZZER_PIN, 0);
    gpio_set_level(LED_R_PIN, 0);
    gpio_set_level(LED_G_PIN, 0);
    gpio_set_level(LED_B_PIN, 0);
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