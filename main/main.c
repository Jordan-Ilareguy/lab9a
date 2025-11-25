#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "esp_adc/adc_oneshot.h"

// -------------------------
// Configuration
// -------------------------
#define WIFI_SSID        "Super Cooper" // set to Electronics
#define WIFI_PASS        "Fr3nch1eBu!!dog" // set to Electr0nic$2024

// set this to your laptop's IPv4 address to publish your Payload to MQTT Broker.
#define MQTT_BROKER_URI  "mqtt://192.168.2.48"

// ADC (ADC1_CH3=GPIO4, ADC1_CH4=GPIO5)
#define JOY_X_CHANNEL    ADC_CHANNEL_3
#define JOY_Y_CHANNEL    ADC_CHANNEL_4
#define JOY_UNIT         ADC_UNIT_1

// Event group used to signal Wi-Fi connection status
static EventGroupHandle_t wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

// Logging tags
static const char *TAG_WIFI     = "WiFi";
static const char *TAG_MQTT     = "MQTT";
static const char *TAG_JOYSTICK = "Joystick";

// Global handles
static esp_mqtt_client_handle_t global_mqtt_client = NULL;
static adc_oneshot_unit_handle_t adc_handle;

// -------------------------
// ADC Initialization
// -------------------------

/* Function Name - init_adc
 * Description   -
 *     Initializes the ADC one-shot driver and configures the channels
 *     used by the joystick X and Y axes.
 * Parameters    - None
 * Return type   - void
 */
static void init_adc() {
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = JOY_UNIT
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOY_X_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOY_Y_CHANNEL, &chan_cfg));
}

// -------------------------
// Joystick MQTT Publish Task
// -------------------------

/* Function Name - joystick_publish_task
 * Description   -
 *     FreeRTOS task that periodically reads joystick X/Y values and
 *     publishes them as a text message to a specific MQTT topic.
 * Purpose       -
 *     To continuously send live joystick sensor data to the MQTT broker
 *     so that other clients can subscribe and monitor the values.
 * Parameters    - None.
 * Return type   - void
 */
static void joystick_publish_task() {
    char message[64];

    while (1) {
        int x_val = 0, y_val = 0;

        // Read joystick X and Y axis values from ADC
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, JOY_X_CHANNEL, &x_val));
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, JOY_Y_CHANNEL, &y_val));

        // Format a simple text message with the values
        snprintf(message, sizeof(message), "Joystick: X=%d, Y=%d", x_val, y_val);

        // Publish only if the MQTT client is valid (connected)
        if (global_mqtt_client) {
            esp_mqtt_client_publish(global_mqtt_client,
                                    "jordan/topic1",
                                    message,
                                    0,    // use string length
                                    0,    // QoS 0 (at most once)
                                    0);   // no retain flag
            ESP_LOGI(TAG_JOYSTICK, "Published: %s", message);
        }

        // Wait 2 seconds before reading and publishing again
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// -------------------------
// MQTT Handler
// -------------------------

/* Function Name - mqtt_event_handler_cb
 * Description   -
 *     Callback function that handles MQTT events such as connection,
 *     incoming data, and disconnection.
 * Purpose       -
 *     To react to MQTT client events:
 *       - When connected: subscribe to a topic, start the joystick task,
 *         and send a greeting.
 *       - When data is received: log it and send an acknowledgement.
 *       - When disconnected: log the status.
 *
 * Parameters    -
 *     void *handler_args
 *         User data passed when registering the event handler (unused).
 *
 *     esp_event_base_t base
 *         Event base (should be MQTT_EVENT).
 *
 *     int32_t event_id
 *         Type of MQTT event (e.g., MQTT_EVENT_CONNECTED).
 *
 *     void *event_data
 *         Pointer to esp_mqtt_event_t structure containing event details.
 *
 * Return type   -
 *     void
 */
static void mqtt_event_handler_cb(void *handler_args,
                                  esp_event_base_t base,
                                  int32_t event_id,
                                  void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    static bool greeted = false;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "Connected to MQTT broker.");

            // Subscribe to the topic to receive joystick or text messages
            esp_mqtt_client_subscribe(event->client, "jordan/topic1", 0);

            // Store client handle globally so the publish task can use it
            global_mqtt_client = event->client;

            // Start the joystick publishing task after connection
            xTaskCreate(joystick_publish_task,
                        "joystick_publish_task",
                        4096,
                        NULL,
                        5,
                        NULL);

            // Send a greeting message only once
            if (!greeted) {
                esp_mqtt_client_publish(event->client,
                                        "jordan/topic1",
                                        "Hello from ESP32 jordan!",
                                        0,
                                        0,
                                        0);
                greeted = true;
            }
            break;

        case MQTT_EVENT_DATA: {
            // Copy incoming data into a local buffer and null-terminate it
            char rx[256] = {0};
            int len = (event->data_len < (int)sizeof(rx) - 1)
                        ? event->data_len
                        : (int)sizeof(rx) - 1;

            memcpy(rx, event->data, len);
            rx[len] = '\0';

            // Log topic and payload
            ESP_LOGI(TAG_MQTT, "Received: topic='%.*s'  data='%s'",
                     event->topic_len, event->topic, rx);

            // If this is not already an acknowledgement, send one back
            if (strncmp(rx, "I receive this:", 14) != 0) {
                char ack[300];
                snprintf(ack, sizeof(ack), "I receive this: %s", rx);
                esp_mqtt_client_publish(event->client,
                                        "jordan/topic1",
                                        ack,
                                        0,
                                        0,
                                        0);
            }
            break;
        }

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG_MQTT, "Disconnected from MQTT broker.");
            break;

        default:
            // Other MQTT events are ignored in this example
            break;
    }
}

/* Function Name - start_mqtt_client
 *
 * Description   -
 *     Creates and starts the MQTT client using the broker URI defined
 *     in the configuration section.
 *
 * Purpose       -
 *     To initialize the MQTT client, register the event handler, and
 *     start the MQTT connection process.
 *
 * Parameters    - None.
 *
 * Return type   - void
 */
static void start_mqtt_client() {
    // Basic MQTT configuration: broker URI only for this example
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    // Initialize MQTT client
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL) {
        ESP_LOGE(TAG_MQTT, "Failed to initialize MQTT client.");
        return;
    }

    // Register event handler and start MQTT client
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client,
                                                   MQTT_EVENT_ANY,
                                                   mqtt_event_handler_cb,
                                                   NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));
    ESP_LOGI(TAG_MQTT, "MQTT client started.");
}

// -------------------------
// Wi-Fi
// -------------------------

/* Function Name - wifi_event_handler
 *
 * Description   -
 *     Event handler for Wi-Fi and IP events (station mode).
 *
 * Purpose       -
 *     To:
 *       - Start connection when Wi-Fi station starts.
 *       - Retry connection when disconnected.
 *       - Set an event bit when an IP address is obtained.
 *
 * Parameters    -
 *     void *arg
 *         User data passed at registration (unused).
 *
 *     esp_event_base_t event_base
 *         Event base (WIFI_EVENT or IP_EVENT).
 *
 *     int32_t event_id
 *         Specific event ID within the event base.
 *
 *     void *event_data
 *         Pointer to data for that event (type depends on event).
 *
 * Return type   - void
 */
static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        // Station started: attempt to connect to the configured AP
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // If disconnected, automatically try to reconnect
        esp_wifi_connect();
        ESP_LOGI(TAG_WIFI, "Retrying connection to Wi-Fi...");

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // When the station gets an IP address, log it and set event bit
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG_WIFI, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* Function Name - wifi_init_sta
 * Description   -
 *     Initializes NVS, network interfaces, Wi-Fi in station mode,
 *     registers event handlers, and starts the Wi-Fi connection.
 * Purpose       -
 *     To fully set up the ESP32 as a Wi-Fi station so that it can
 *     connect to the specified access point and obtain an IP address.
 * Parameters    - None.
 * Return type   - void
 */
static void wifi_init_sta() {
    // ----- NVS initialization -----
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // ----- Network interface + event loop -----
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // ----- Wi-Fi driver initialization -----
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // ----- Register event handlers BEFORE starting Wi-Fi -----
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // Create event group to track connection status
    wifi_event_group = xEventGroupCreate();

    // ----- Configure Wi-Fi station parameters -----
    wifi_config_t wifi_config = (wifi_config_t){0};

    // Copy SSID and password into config (ensure null-termination)
    strncpy((char *)wifi_config.sta.ssid,
            WIFI_SSID,
            sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password,
            WIFI_PASS,
            sizeof(wifi_config.sta.password) - 1);

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    // ----- Apply Wi-Fi configuration and start station mode -----
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_WIFI, "Wi-Fi init complete. Connecting to SSID: %s", WIFI_SSID);
}

// -------------------------
// Main
// -------------------------
/* 
 *  1. Initialize and connect to Wi-Fi.
 *  2. Wait until Wi-Fi is connected.
 *  3. Initialize ADC for joystick.
 *  4. Start the MQTT client.
 */
void app_main() {
    // Initialize and start Wi-Fi in station mode
    wifi_init_sta();

    // Wait indefinitely until we get the "Wi-Fi connected" bit
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_WIFI, "Connected to Wi-Fi network: %s", WIFI_SSID);

        // After successful Wi-Fi connection, set up ADC and MQTT
        init_adc();
        start_mqtt_client();
    } else {
        // This branch should not normally happen with portMAX_DELAY
        ESP_LOGW(TAG_WIFI, "Failed to connect to Wi-Fi.");
    }
}
