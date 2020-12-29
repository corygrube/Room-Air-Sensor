/* UART asynchronous example, that uses separate RX and TX tasks
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"

#define UART_NUM UART_NUM_1
#define BUF_SIZE 1024
#define TXD_PIN 4
#define RXD_PIN 5

void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_DEBUG);
    while (1) {
        const char cmd[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
		const int txBytes = uart_write_bytes(UART_NUM, (const char*) cmd, 9);
		ESP_LOGD(TX_TASK_TAG, "Wrote %d bytes", txBytes);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_DEBUG);
    uint8_t* data = (uint8_t*) malloc(9);
    while (1) {
        int len = 0;
        uart_get_buffered_data_len(UART_NUM, (size_t *)&len);
        if (len > 8) {
            const int rxBytes = uart_read_bytes(UART_NUM, data, 9, 0);
        
            if (rxBytes > 0) {
                ESP_LOGD(RX_TASK_TAG, "Read %d bytes: '%x'", rxBytes, (unsigned int) data);
                ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_DEBUG);
                const int CO2 = data[2] * 256 + data[3];
                ESP_LOGI(RX_TASK_TAG, "Current CO2 measurement: %d ppm", CO2);
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    free(data);
}

void app_main(void)
{
    init();
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, 2, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, 1, NULL);
}