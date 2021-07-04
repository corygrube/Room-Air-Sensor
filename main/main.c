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
#define UART_NUM UART_NUM_1     // Reserving UART0 for USB comms, so UART 1 was used
#define BUF_SIZE 1024
#define TXD_PIN 4
#define RXD_PIN 5
#define DHT11_PIN GPIO_NUM_0

/*
 * Signal bits used for program flow control
 * FreeRTOS event group manages/gets bit states
 * Custom bit names mapped to pre-defined BITx registers
 */
static EventGroupHandle_t s_event_group;
#define WIFI_CONNECTED_BIT  BIT0    // wifi connected and IP assigned
#define DHT11_OK_BIT        BIT1    // Last DHT11 read was successful
#define CO2_OK_BIT          BIT2    // Last CO2 read was successful

// default logger
static const char *LOG = "Logger";

/*
 * Function: wifi_event_handler
 * -----
 * Description:
 *      Event handler for all WiFi events.
 *      Utilized on WiFi startup, connected, and disconnected.
 *      Attempts to maintain WiFi connection at all times and manage
 *          WIFI_CONNECTED_BIT event group bit.
 * 
 * Arguments:
 *      arg: Data passed to event handler (null in this case)
 *      event_base: always WIFI_EVENT
 *      event_id: the specific WIFI_EVENT triggering the handler
 *      event_data: data specific to the event
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_STA_START) {
        // Attempt initial connection
        ESP_LOGD(LOG, "Wifi started successfully. Attempting initial connection.");
        esp_err_t ret = esp_wifi_connect();
        if (ret != ESP_OK) {
            ESP_LOGE(LOG, "%s", esp_err_to_name(ret));
        }
    
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // Clear connected status bit. Attempt to reconnect after 30s 
        ESP_LOGW(LOG, "Wifi disconnected or connection attempt failed. Retrying connection in 30s.");
        xEventGroupClearBits(s_event_group, WIFI_CONNECTED_BIT);
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


/*
 * Function: ip_event_handler
 * -----
 * Description:
 *      event handler for all IP events.
 *      Utilized to set WIFI_CONNECTED_BIT event group bit
 *          when IP address is assigned.
 * 
 * Arguments:
 *      arg: Data passed to event handler (null in this case)
 *      event_base: always IP_EVENT
 *      event_id: the specific IP_EVENT triggering the handler
 *      event_data: data specific to the event
 */
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data)
{
    if (event_id == IP_EVENT_STA_GOT_IP) {
        // Log IP info and set event group bits.
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(LOG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_event_group, WIFI_CONNECTED_BIT);
    }
}

/*
 * Function: system_init
 * -----
 * Description:
 *      Creates event group.
 *      Creates default event loop.
 *      Registers wifi_event_handler and ip_event_handler functions.
 */
void system_init(void)
{
    ESP_LOGD(LOG, "System initialization started");
    s_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
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
    ESP_LOGD(LOG, "System initialization complete");
}

/*
 * Function: sensor_init
 * -----
 * Description:
 *      Initializes and configures UART for MH-Z19B (CO2) reads
 *      Runs DHT11 init function
 */
void sensor_init(void)
{
	// inits starting 
	ESP_LOGD(LOG, "Sensor initialization started");

	// UART config/init
	const uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0,
                                        NULL, 0));
	ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	ESP_LOGD(LOG, "UART initialized on TX/RX pins %d, %d", TXD_PIN, RXD_PIN);

	// DHT11 init
	DHT11_init(DHT11_PIN);
	ESP_LOGD(LOG, "DHT11 initialized on pin %d", DHT11_PIN);

	// inits complete
	ESP_LOGD(LOG, "Sensor initialization complete");
}

/*
 * Function: wifi_init
 * -----
 * Description:
 *      Initializes and configures WiFi
 */
void wifi_init(void)
{
	// Wifi inits starting 
	ESP_LOGD(LOG, "Wifi initialization started");
    
    // Initialize non-volatile storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // init TCP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // create default wifi station
    esp_netif_create_default_wifi_sta();

    // init wifi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));    

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

    ESP_LOGD(LOG, "Wifi initialization complete");

    // Note that Wifi is not conected yet, just initialized/started
}

/*
 * Function: get_co2
 * -----
 * Description:
 *      Reads current CO2 measurement from MH-Z19B sensor.
 *      3 step process: UART write command, UART read response,
 *          calculate CO2 from response
 *      Event group bits denote whether most recent reading is valid.
 * 
 * Returns:
 *      CO2 measurement in ppm (0-5000)
 *          On error, returns -1
 */
int get_co2(void)
{
	/* UART Prep */
    // flush UART buffer data
	uart_flush(UART_NUM);
	ESP_LOGD(LOG, "UART buffer flushed ahead of CO2 read");

	/* UART TX */
	// hard-coded command for CO2 measurement - see documentation for more info
	const char cmd[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
	// UART write
	const int txBytes = uart_write_bytes(UART_NUM, (const char*) cmd, 9);
	ESP_LOGD(LOG, "Wrote %d bytes", txBytes);

	// Wait 100ms for response
	vTaskDelay(100 / portTICK_PERIOD_MS);
	
    /* UART RX */
	// Assume buffer has only 9 bytes
	uint8_t* data = (uint8_t*) malloc(9);
	const int rxBytes = uart_read_bytes(UART_NUM, data, 9, 0);
	int co2 = 0;
	
    // Check if read had correct number of bytes (9=successful):
	if (rxBytes == 9) {
		ESP_LOGD(LOG, "Read %d bytes: '%x'", rxBytes, (unsigned int) data);
		ESP_LOG_BUFFER_HEXDUMP(LOG, data, rxBytes, ESP_LOG_DEBUG);
		// calculate CO2. Byte 2 is high byte, Byte 3 is low byte
		co2 = data[2] * 256 + data[3];
		ESP_LOGI(LOG, "New CO2 reading: %d ppm", co2);
        // Update event group bit
        xEventGroupSetBits(s_event_group, CO2_OK_BIT);
	
    } else {
        // log warning for read fault, set on-error value (-1)
		co2 = -1;
		ESP_LOGW(LOG, "CO2 Read Fault");
        // update event group bit
        xEventGroupClearBits(s_event_group, CO2_OK_BIT);
	}
	
    // free malloc and return
	free(data);
	return co2;
}

/*
 * Function: dht11_reading
 * -----
 * Description:
 *      Reads current temp/humidity measurement from DHT11 sensor.
 *      status enum used to determine data validity.
 *      Event group bits denote whether most recent reading is valid.
 * 
 * Returns:
 *      dht11_reading struct
 *          *.temperature in DegC
 *          *.humidity in %RH
 *          *.status enum (0=OK, -1=TIMEOUT_ERROR, -2=CRC_ERROR)
 *          
 *          On error, returns -1 for temp/humidity
 */
struct dht11_reading get_dht11(void)
{
    struct dht11_reading data = DHT11_read();
    // Check if read was successful
    if (data.status == DHT11_OK) {
        // Log new data
        ESP_LOGI(LOG, "New temperature reading: %d°C",
                 data.temperature);
        ESP_LOGI(LOG, "New humidity reading: %d%%RH",
                 data.humidity);
        // Update event group bit
        xEventGroupSetBits(s_event_group, DHT11_OK_BIT);
    } else {
        // Log warning and cause of read fault
        ESP_LOGW(LOG, "DHT11 read fault.");
        if (data.status == DHT11_CRC_ERROR) {
            ESP_LOGW(LOG, "Cause of Fault: CRC Error");
        } else if (data.status == DHT11_TIMEOUT_ERROR) {
            ESP_LOGW(LOG, "Cause of Fault: Read Timeout");
        } else {
            ESP_LOGW(LOG, "Cause of Fault: Unknown");
        }
        // Update event group bit
        xEventGroupClearBits(s_event_group, DHT11_OK_BIT);
    }
    return data;
}

/*
 * Function: app_main
 * -----
 * Description:
 *      Main loop.
 *      Runs init functions, reads data, and transmits valid data over MQTT
 */
void app_main(void)
{
	ESP_LOGI(LOG, "Program started");

	// Run initialization functions
	system_init();
    sensor_init();
    wifi_init();

	// main loop
	while (1) {
		ESP_LOGD(LOG, "Program loop started");
		
        // Read DHT11 (Temperature/Humidity)
        struct dht11_reading dht11_data = get_dht11();
        
        // Read MH-Z19B (CO2)
		int co2 = get_co2();

        // Check if Wifi is connected
        EventBits_t bits = xEventGroupGetBits(s_event_group);
        if (bits & WIFI_CONNECTED_BIT) {
            // MQTT logic probably maybe possibly
            // May roll sensor reads into here. depends on how MQTT works
            if (bits & DHT11_OK_BIT) {
                // MQTT Pub logic
            }
            if (bits & CO2_OK_BIT) {
                // MQTT Pub logic
            }
        } else {
            // MQTT skip/queue logic, i guess? may not be necessary
        }

		// wait 5s before repeating
		ESP_LOGD(LOG, "Program loop waiting");
		vTaskDelay(5000 / portTICK_PERIOD_MS);
		ESP_LOGD(LOG, "Program loop completed");
	}
}
