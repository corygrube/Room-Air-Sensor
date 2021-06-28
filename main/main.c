// #define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "dht11.h"

#define UART_NUM UART_NUM_1
#define BUF_SIZE 1024
#define TXD_PIN 4
#define RXD_PIN 5
#define DHT11_PIN GPIO_NUM_0

// Define default logger
static const char *LOG = "Logger";

// hardware/protocol initializaiton. Pins defined in globals
void init(void) {
	// inits starting 
	ESP_LOGI(LOG, "Initialization started");

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
	ESP_LOGI(LOG, "Initialization complete");
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
	
	// initialize UART, DHT11
	init();

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
