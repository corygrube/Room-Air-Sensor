#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "string.h"

#define UART_NUM UART_NUM_1
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define UART_BUF 1024
static const char *TAG = "uart";
static QueueHandle_t uart_queue;

// static void uart_task(void *pvParameters) {
// 	// Configure a temporary buffer for the incoming UART data
//     uint8_t *data = (uint8_t *) malloc(UART_BUF);

// 	while(1) {
// 		// Read data from the UART
//         int len = uart_read_bytes(UART_NUM, data, UART_BUF, 20 / portTICK_RATE_MS);
//         // Write data back to the UART
//         uart_write_bytes(UART_NUM, (const char *) data, len);
// 	}
// }

void app_main(void) {	
	// set UART log level
	esp_log_level_set(TAG, ESP_LOG_INFO);
	
	// configure UART
	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		// .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 122,
	};
	ESP_ERROR_CHECK(
		uart_param_config(
			UART_NUM,
			&uart_config
		)
	);

	// set UART pins
	ESP_ERROR_CHECK(
		uart_set_pin(
			UART_NUM,
			UART_TX_PIN,
			UART_RX_PIN,
			UART_PIN_NO_CHANGE,
			UART_PIN_NO_CHANGE
		)
	);
	
	// install UART driver
	uart_driver_install(
		UART_NUM,
		UART_BUF * 2,
		UART_BUF * 2,
		10,
		&uart_queue,
		0
	);

	ESP_LOGI(TAG, "UART configured.");

	// Configure a temporary buffer for the incoming UART data
    uint8_t *data = (uint8_t *) malloc(UART_BUF);

	while(1) {
		// Write data to UART
		const char cmd[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x79};
		// const char* tx = cmd
		const int txBytes = uart_write_bytes(UART_NUM, (const char*) cmd, 8);
		ESP_LOGI(TAG, "Wrote %d bytes.\n", txBytes);
		for (int i=0; i<8; ++i) {
			printf("r%d: '%x', ", i, cmd[i]);
		}
		printf("\n");

		// wait 1s
		// vTaskDelay(1000 / portTICK_PERIOD_MS);
		ets_delay_us(1000000);

		// Read data from the UART
        const int rxBytes = uart_read_bytes(UART_NUM, data, 8, 1000 / portTICK_PERIOD_MS);
		// wait 1s
		// vTaskDelay(1000 / portTICK_PERIOD_MS);
		// ets_delay_us(1000000);
		ESP_LOGI(TAG, "Read %d bytes.\n", rxBytes);
		for (int i=0; i<8; ++i) {
			printf("w%d: '%x', ", i, data[i]);
		}
		printf("\n");

		// wait 1s
		// vTaskDelay(1000 / portTICK_PERIOD_MS);
		ets_delay_us(1000000);
	}

	// printf("Hello world!\n");

	// /* Print chip information */
	// esp_chip_info_t chip_info;
	// esp_chip_info(&chip_info);
	// printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
	//         chip_info.cores,
	//         (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
	//         (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

	// printf("silicon revision %d, ", chip_info.revision);

	// printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
	//         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	// for (int i = 10; i >= 0; i--) {
	//     printf("Restarting in %d seconds...\n", i);
	//     vTaskDelay(1000 / portTICK_PERIOD_MS);
	// }
	// printf("Restarting now.\n");
	// fflush(stdout);
	// esp_restart();
}
