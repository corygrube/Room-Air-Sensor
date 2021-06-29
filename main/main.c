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
#include "dht11.h"
#include "lwip/err.h"
#include "lwip/sys.h"

// sensor definitions
#define UART_NUM UART_NUM_1
#define BUF_SIZE 1024
#define TXD_PIN 4
#define RXD_PIN 5
#define DHT11_PIN GPIO_NUM_0

/* The examples use WiFi configuration that you can set via project configuration menu
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

// Define default logger
static const char *LOG = "Logger";

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(LOG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(LOG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(LOG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
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
	ESP_LOGI(LOG, "Wifi initialization complete");
}

// Wifi initialization
void wifi_init(void) {
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

    // register event handler instances for WIFI_EVENTs/IP_EVENT
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // define wifi config from menuconfig params
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
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

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(LOG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(LOG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(LOG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

// MH-Z19B read CO2
// Combines both TX/RX UART steps
// Lazy implementation, async implementation possible in future
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

	// [LAZY/BAD PRACTICE] Wait 1s for response
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	// UART RX
	// [LAZY/BAD PRACTICE] Assume all 9 bytes are in
	// read-buffer allocation
	uint8_t* data = (uint8_t*) malloc(9);
	// read 9 byte response, init co2
	const int rxBytes = uart_read_bytes(UART_NUM, data, 9, 0);
	int co2 = 0;
	// if read was successful:
	if (rxBytes > 0) {
		// log read
		ESP_LOGD(LOG, "Read %d bytes: '%x'", rxBytes, (unsigned int) data);
		ESP_LOG_BUFFER_HEXDUMP(LOG, data, rxBytes, ESP_LOG_DEBUG);
		// calculate and log CO2. Byte 2 is high byte, Byte 3 is low byte
		co2 = data[2] * 256 + data[3];
		ESP_LOGI(LOG, "CO2 read successfully. Current: %d ppm", co2);
	}
	// if read unsuccessful:
	else {
		co2 = -1;
		ESP_LOGI(LOG, "Problem Reading CO2");
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

		// repeat every 5s
		ESP_LOGI(LOG, "Program loop waiting");
		vTaskDelay(5000 / portTICK_PERIOD_MS);
		ESP_LOGI(LOG, "Program loop completed");
	}
}
