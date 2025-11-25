#include <stdio.h>
#include <inttypes.h>
#include <string.h>                                  
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"      // Used for Wi-Fi connection flag
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_wifi.h"                               // Wi-Fi functions
#include "esp_event.h"                              // Wi-Fi/MQTT events
#include "esp_netif.h"                              // Network interface
#include "nvs_flash.h"                              // Required for Wi-Fi
#include "mqtt_client.h"                            // MQTT functions

// ---------------- I2C Definitions ----------------
#define I2C_MASTER_SCL_IO           37         // SCL pin
#define I2C_MASTER_SDA_IO           38         // SDA pin
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000     // 400 kHz
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define MPU_TEMP                    0x41       // Temperature register
#define MPU6050_ADDR                0x68       // AD0 = GND
#define MPU6050_WHO_AM_I_REG        0x75
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_ACCEL_XOUT_H        0x3B       // Start of accel data
#define I2C_TIMEOUT_MS              1000

// -------- Wi-Fi / MQTT config --------
#define WIFI_SSID        "Electronics"
#define WIFI_PASS        "Electr0nic$2024"
#define MQTT_BROKER_URI  "mqtt://192.168.117.17"
#define MQTT_TOPIC       "jordan/topic2"

static const char *TAG = "MPU6050";
static const char *TAG_WIFI     = "WiFi"; // Wi-Fi tag
static const char *TAG_MQTT     = "MQTT"; // MQTT tag

// Function prototype to read bytes (JI)
esp_err_t i2c_read_bytes(uint8_t device_addr, uint8_t start_reg, uint8_t *buffer, size_t length); // I2C read function

// Event group: signals when Wi-Fi is connected
static EventGroupHandle_t wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;  // Bit flag = Wi-Fi ready

// Global MQTT client handle
static esp_mqtt_client_handle_t g_mqtt = NULL; // MQTT client handle

// ---------------- I2C Master Init ----------------
esp_err_t i2c_master_init()
{
    // Configure I2C master mode, pins, and speed
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));

     // Install I2C driver
    esp_err_t ret = i2c_driver_install(I2C_MASTER_NUM,
                              conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE,
                              0);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake MPU6050: %s", esp_err_to_name(ret)); // Log error
        return ret;
    }

    // Wake MPU6050 from sleep
    uint8_t wake[2] = {MPU6050_PWR_MGMT_1, 0x00}; // PWR_MGMT_1 = 0x00 (wake up)
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM,
                                               MPU6050_ADDR,
                                                wake, 
                                               sizeof(wake),
                                               pdMS_TO_TICKS(I2C_TIMEOUT_MS)));

                                               if (ret!= ESP_OK) {
                                                    ESP_LOGE(TAG, "Failed to wake MPU6050: %s", esp_err_to_name(ret));
                                                    return ret;
                                            
}

ESP_LOGI(TAG, "MPU6050 woken up successfully");
    
    // Give sensor time to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    return ESP_OK;
}

// ---------------- Wi-Fi Event Handler ----------------
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();  // Try connecting to AP

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();  // Auto reconnect
        ESP_LOGI(TAG, "WiFi disconnected, retrying…");

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        // IP received → Wi-Fi is now fully connected
        const ip_event_got_ip_t *e = (const ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));

        // Set Wi-Fi connected flag
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ---------------- Wi-Fi Initialization ----------------
static void wifi_init_sta()
{
    // Init NVS (required for Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Init network stack + default event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Initialize Wi-Fi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers BEFORE Wi-Fi starts
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // Create event group for Wi-Fi status
    wifi_event_group = xEventGroupCreate();

    // Set Wi-Fi credentials
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    // Start Wi-Fi in station mode
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi init done. Connecting to %s…", WIFI_SSID);
}

// ---------------- MQTT Event Handler ----------------
static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            g_mqtt = event->client;  // Save MQTT client handle
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT disconnected");
            g_mqtt = NULL;
            break;

        default:
            break;
    }
}

// ---------------- Start MQTT Client ----------------
static void mqtt_start()
{
    // Configure MQTT broker URI
    const esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    // Create MQTT client
    esp_mqtt_client_handle_t c = esp_mqtt_client_init(&cfg);

    // Register MQTT event handler
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(c, MQTT_EVENT_ANY, mqtt_event_handler, NULL));

    // Start MQTT service
    ESP_ERROR_CHECK(esp_mqtt_client_start(c));

    ESP_LOGI(TAG, "MQTT client started: %s", MQTT_BROKER_URI);
}

// ---------- Task: Read MPU6050 & Publish JSON ----------  
static void mpu6050_publish_task(void *arg)
{
    while (1) {
        uint8_t data[14];

        // Read 14 bytes: accel(6), temp(2), gyro(6)
        if (i2c_read_bytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, data, sizeof(data)) == ESP_OK) {

            // Convert raw sensor values
            int16_t ax = (data[0] << 8) | data[1];
            int16_t ay = (data[2] << 8) | data[3];
            int16_t az = (data[4] << 8) | data[5];

            int16_t temp_raw = (data[6] << 8) | data[7];
            float temperature = (temp_raw / 340.0f) + 36.53f;

            int16_t gx = (data[8] << 8)  | data[9];
            int16_t gy = (data[10] << 8) | data[11];
            int16_t gz = (data[12] << 8) | data[13];

            // Build JSON message
            char payload[160];
            snprintf(payload, sizeof(payload),
                    "{\"ax\":%d,\"ay\":%d,\"az\":%d,\"temp_c\":%.2f,\"gx\":%d,\"gy\":%d,\"gz\":%d}",
                    ax, ay, az, temperature, gx, gy, gz);

            ESP_LOGI(TAG, "MPU TX: %s", payload);

            // Publish to MQTT topic
            if (g_mqtt) {
                esp_mqtt_client_publish(g_mqtt,
                                    "jordan/topic2",
                                    payload,
                                    0,    // use string length
                                    0,    // QoS 0 (at most once)
                                    0);   // no retain flag
            ESP_LOGI(TAG_MQTT, "Published: %s", payload);
            }

        } else {
            ESP_LOGW(TAG, "Failed to read MPU6050 data");  // Warning message
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Publish every 1s
    }
}


void app_main()
{
    // Step 1: Initialize I2C

    ESP_ERROR_CHECK(i2c_master_init()); // Initialize I2C
    ESP_LOGI(TAG, "I2C initialized"); // I2C init done

    // Step 2: Initialize wifi

    wifi_init_sta();
    ESP_LOGI(TAG_WIFI, "Wi-Fi init started");

// Wait indefinitely until we get the "Wi-Fi connected" bit
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {

        ESP_LOGI(TAG_WIFI, "Wi-Fi connected successfully"); // Connected to Wi-Fi

        mqtt_start(); // Start MQTT client

        // Start publisher task
        xTaskCreate(mpu6050_publish_task, "mpu6050_publish_task", 4096, NULL, 5, NULL);
    }

}

/* ---------- I2C read function definition from Lab 8----------*/

/* Function Name - i2c_read_bytes

* Description - This function reads a block of data from an I2C device.

* Return type - The return type is esp_err_t, which indicates the success or failure of the operation.

* Parameters - 

- parameter1 - uint8_t device_addr: The I2C address of the device to read from.

- parameter2 - uint8_t start_reg: The starting register address to read from.

- parameter3 - uint8_t *buffer: A pointer to the buffer to store the read data.

- parameter4 - size_t length: The number of bytes to read.

*/

esp_err_t i2c_read_bytes(uint8_t device_addr, uint8_t start_reg, uint8_t *buffer, size_t length) // I2C read function definition
{
    return i2c_master_write_read_device(I2C_MASTER_NUM,
                                        device_addr,
                                        &start_reg,
                                        1,   
                                        buffer,
                                        length,                                                                                                     
                                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}