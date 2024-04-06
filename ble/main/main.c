/*	BLE SPP Server Example

	This example code is in the Public Domain (or CC0 licensed, at your option.)

	Unless required by applicable law or agreed to in writing, this
	software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/uart.h"

#include "cmd.h"

static const char *TAG = "MAIN";

QueueHandle_t xQueueSpp;
QueueHandle_t xQueueUart;

#define RX_BUF_SIZE	128

void uart_init(void) {
	const uart_config_t uart_config = {
		//.baud_rate = 115200,
		.baud_rate = CONFIG_UART_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
		.source_clk = UART_SCLK_DEFAULT,
#else
		.source_clk = UART_SCLK_APB,
#endif
	};
	// We won't use a buffer for sending data.
	uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
	uart_param_config(UART_NUM_1, &uart_config);
	//uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_set_pin(UART_NUM_1, CONFIG_UART_TX_GPIO, CONFIG_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void uart_tx_task(void* pvParameters)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start using GPIO%d", CONFIG_UART_TX_GPIO);
	CMD_t cmdBuf;

	while(1) {
		xQueueReceive(xQueueUart, &cmdBuf, portMAX_DELAY);
		ESP_LOGI(pcTaskGetName(NULL), "cmdBuf.length=%d", cmdBuf.length);
		ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), cmdBuf.payload, cmdBuf.length, ESP_LOG_INFO);
		int txBytes = uart_write_bytes(UART_NUM_1, cmdBuf.payload, cmdBuf.length);
		if (txBytes != cmdBuf.length) {
			ESP_LOGE(pcTaskGetName(NULL), "uart_write_bytes Fail. txBytes=%d cmdBuf.length=%d", txBytes, cmdBuf.length);
		}
	} // end while

	// Never reach here
	vTaskDelete(NULL);
}

static void uart_rx_task(void* pvParameters)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start using GPIO%d", CONFIG_UART_RX_GPIO);
	CMD_t cmdBuf;
	cmdBuf.spp_event_id = BLE_UART_EVT;
	while (1) {
		cmdBuf.length = uart_read_bytes(UART_NUM_1, cmdBuf.payload, PAYLOAD_SIZE, 10 / portTICK_PERIOD_MS);
		// There is some rxBuf in rx buffer
		if (cmdBuf.length > 0) {
			ESP_LOGD(pcTaskGetName(NULL), "cmdBuf.length=%d", cmdBuf.length);
			ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), cmdBuf.payload, cmdBuf.length, ESP_LOG_DEBUG);
			BaseType_t err = xQueueSend(xQueueSpp, &cmdBuf, portMAX_DELAY);
			if (err != pdTRUE) {
				ESP_LOGE(pcTaskGetName(NULL), "xQueueSend Fail");
			}
		} else {
			// There is no data in rx buufer
			//ESP_LOGI(pcTaskGetName(NULL), "Read %d", rxBytes);
		}
	} // end while

	// Never reach here
	vTaskDelete(NULL);
}

void spp_task(void * arg);

void app_main(void)
{
	// Initialize NVS.
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );

	// Create Queue
	xQueueSpp = xQueueCreate(10, sizeof(CMD_t));
	configASSERT( xQueueSpp );
	xQueueUart = xQueueCreate( 10, sizeof(CMD_t) );
	configASSERT( xQueueUart );

	// uart initialize
	uart_init();

	// Start tasks
	xTaskCreate(uart_tx_task, "UART-TX", 1024*4, NULL, 2, NULL);
	xTaskCreate(uart_rx_task, "UART-RX", 1024*4, NULL, 2, NULL);
	xTaskCreate(spp_task, "SPP", 1024*4, NULL, 2, NULL);
}
