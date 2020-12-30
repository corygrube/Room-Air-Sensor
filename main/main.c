#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"

#define UART_NUM UART_NUM_1
#define BUF_SIZE 1024
#define TXD_PIN 4
#define RXD_PIN 5

// UART initializaiton. UART controller/pins defined above
void init(void) {
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
}

// UART (CO2) write task
static void tx_task(void *arg) {
	// define debug logger
	static const char *TX_TASK_TAG = "TX_TASK";
	esp_log_level_set(TX_TASK_TAG, ESP_LOG_DEBUG);
	
	// main task loop
	while (1) {
		// hard-coded command for CO2 measurement
		const char cmd[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
		
		// UART write
		const int txBytes = uart_write_bytes(UART_NUM, (const char*) cmd, 9);
		ESP_LOGD(TX_TASK_TAG, "Wrote %d bytes", txBytes);
		
		// Execute every 5s. sensor doesn't change reading faster than that
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}

// UART (CO2) read task
static void rx_task(void *arg) {
	// define debug logger
	static const char *RX_TASK_TAG = "RX_TASK";
	esp_log_level_set(RX_TASK_TAG, ESP_LOG_DEBUG);
	
	// read buffer allocation
	uint8_t* data = (uint8_t*) malloc(9);
	
	// main task loop
	while (1) {
		// only read when buffer is at least 9 bytes (length of one response)
		int len = 0;
		uart_get_buffered_data_len(UART_NUM, (size_t *)&len);
		if (len > 8) {
			// UART read
			const int rxBytes = uart_read_bytes(UART_NUM, data, 9, 0);

			// if read was successful:
			if (rxBytes > 0) {
				// log read
				ESP_LOGD(RX_TASK_TAG, "Read %d bytes: '%x'", rxBytes, (unsigned int) data);
				ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_DEBUG);
				
				// calculate and log CO2. Byte 2 is high byte, Byte 3 is low byte
				const int CO2 = data[2] * 256 + data[3];
				ESP_LOGI(RX_TASK_TAG, "Current CO2 measurement: %d ppm", CO2);
			}
		}
		// check for reads every second
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	// free malloc. just here for best practices, but we're in bad news city if this happens
	free(data);
}

// startup
void app_main(void) {
	// start UART, read/write tasks
	init();
	xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, 2, NULL);
	xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, 1, NULL);
}