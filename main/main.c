#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "dht11.h"

// sensor definitions
#define UART_NUM UART_NUM_1
#define BUF_SIZE 1024
#define TXD_PIN 4
#define RXD_PIN 5
#define DHT11_PIN GPIO_NUM_0

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP */
#define WIFI_CONNECTED_BIT  BIT0

// Define default logger
static const char *LOG = "Logger";

// Wifi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_STA_START) {
        // attempt initial connection
        ESP_LOGI(LOG, "Wifi started successfully. Attempting initial connection.");
        esp_err_t ret = esp_wifi_connect();
        if (ret != ESP_OK) {
            ESP_LOGE(LOG, "%s", esp_err_to_name(ret));
        }
    
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // Clear connected status bit. Attempt to reconnect 
        ESP_LOGE(LOG, "Wifi disconnected or connection attempt failed. Retrying connection in 30s.");
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        vTaskDelay(30000 / portTICK_PERIOD_MS);
        esp_err_t ret = esp_wifi_connect();
        if (ret != ESP_OK) {
            ESP_LOGE(LOG, "%s", esp_err_to_name(ret));
        }

    } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
        // Log connection success. Don't change connected status bit until IP is obtained
        ESP_LOGI(LOG, "Wifi connected successfully.");
    }
}

// IP event handler
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data)
{
    if (event_id == IP_EVENT_STA_GOT_IP) {
        // Log IP info and set event group bits. Reset retry number.
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(LOG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// sensor hardware/protocol initializaiton. Pins defined in globals
void sensor_init(void) {
	// inits starting 
	ESP_LOGI(LOG, "Sensor initialization started");

	// UART config/init
	const uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};
	uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
	uart_param_config(UART_NUM, &uart_config);
	uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	ESP_LOGI(LOG, "UART initialized on TX/RX pins %d, %d", TXD_PIN, RXD_PIN);

	// DHT11 init
	DHT11_init(DHT11_PIN);
	ESP_LOGI(LOG, "DHT11 initialized on pin %d", DHT11_PIN);

	// inits complete
	ESP_LOGI(LOG, "Sensor initialization complete");
}

// Wifi initialization
void wifi_init(void) {
	// Wifi inits starting 
	ESP_LOGI(LOG, "Wifi initialization started");
    
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // create event group
	s_wifi_event_group = xEventGroupCreate();

    // TCP stack init
    ESP_ERROR_CHECK(esp_netif_init());

    // create default event loop, default wifi station
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // init wifi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // register event handler instances for WIFI_EVENT/IP_EVENT events
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &ip_event_handler,
                                                        NULL,
                                                        NULL));

    // define wifi config from menuconfig params
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    // Set wifi mode to station, configure, and start
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(LOG, "Wifi initialization complete");

    // Note that Wifi is not conected yet, just initialized/started
}

// MH-Z19B read CO2
// Combines both TX/RX UART steps
// async implementation possible in future
// Returns CO2 integer in ppm (0-5000)
// If read is unsuccessful, returns -1
int get_co2(void) {
	// flush UART buffer data
	uart_flush(UART_NUM);
	ESP_LOGD(LOG, "UART buffer flushed ahead of CO2 read");

	// UART TX
	// hard-coded command for CO2 measurement - see documentation for more info
	const char cmd[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
	// UART write
	const int txBytes = uart_write_bytes(UART_NUM, (const char*) cmd, 9);
	ESP_LOGD(LOG, "Wrote %d bytes", txBytes);

	// Wait 100ms for response
	vTaskDelay(100 / portTICK_PERIOD_MS);

	// UART RX
	// Assume buffer has only 9 bytes are in (trust but verify)
	// read-buffer allocation
	uint8_t* data = (uint8_t*) malloc(9);
	// read 9 byte response, init co2
	const int rxBytes = uart_read_bytes(UART_NUM, data, 9, 0);
	int co2 = 0;
	
    // Check if read had correct number of bytes (9=successful):
	if (rxBytes == 9) {
		// log read bytes
		ESP_LOGD(LOG, "Read %d bytes: '%x'", rxBytes, (unsigned int) data);
		ESP_LOG_BUFFER_HEXDUMP(LOG, data, rxBytes, ESP_LOG_DEBUG);
		// calculate and log CO2. Byte 2 is high byte, Byte 3 is low byte
		co2 = data[2] * 256 + data[3];
		ESP_LOGI(LOG, "CO2 read successfully. Current: %d ppm", co2);
	
    } else {
        // log error and set invalid reading
		co2 = -1;
		ESP_LOGE(LOG, "Problem Reading CO2");
	}
	
    // free malloc and return
	free(data);
	return co2;
}

// DHT11 read temperature
// Returns temperature integer in DegC
int get_temp(void) {
	const int temp = DHT11_read().temperature;
	ESP_LOGI(LOG, "Temperature read successfully. Current: %d°C", temp);
	return temp;
}

// DHT11 read humidity
// Returns humidity integer in %RH
int get_humidity(void) {
	const int humidity = DHT11_read().humidity;
	ESP_LOGI(LOG, "Humidity read successfully. Current: %d%%", humidity);
	return humidity;
}

// Main
void app_main(void) {
	// log program start
	ESP_LOGI(LOG, "Program started");

	// initialize sensor hardware/protocols (UART, DHT11)
	sensor_init();

	// initialize wifi
	wifi_init();

	// main loop
	while (1) {
		ESP_LOGI(LOG, "Program loop started");
		int temp = get_temp();
		int humidity = get_humidity();
		int co2 = get_co2();

        // Check if Wifi is connected
        EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
        if (bits & WIFI_CONNECTED_BIT) {
            // MQTT logic probably maybe possibly
            // May roll sensor reads into here. depends on how MQTT works
        
        } else {
            // MQTT skip/queue logic, i guess? may not be necessary
        }

		// wait 5s before repeating
		ESP_LOGI(LOG, "Program loop waiting");
		vTaskDelay(5000 / portTICK_PERIOD_MS);
		ESP_LOGI(LOG, "Program loop completed");
	}
}
