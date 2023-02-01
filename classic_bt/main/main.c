/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/message_buffer.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "driver/uart.h"


#include "cmd.h"

#define TAG "MAIN"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "ESP_SPP_ACCEPTOR"

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

QueueHandle_t xQueueMain;
QueueHandle_t xQueueUart;
TimerHandle_t timerHandle;

static char *bda2str(uint8_t * bda, char *str, size_t size)
{
	if (bda == NULL || str == NULL || size < 18) {
		return NULL;
	}

	uint8_t *p = bda;
	sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
			p[0], p[1], p[2], p[3], p[4], p[5]);
	return str;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
	char bda_str[18] = {0};
	CMD_t cmdBuf;

	switch (event) {
	case ESP_SPP_INIT_EVT:
		if (param->init.status == ESP_SPP_SUCCESS) {
			ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
			esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
		} else {
			ESP_LOGE(TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
		}
		break;
	case ESP_SPP_DISCOVERY_COMP_EVT:
		ESP_LOGI(TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
		break;
	case ESP_SPP_OPEN_EVT:
		ESP_LOGI(TAG, "ESP_SPP_OPEN_EVT");
		break;
	case ESP_SPP_CLOSE_EVT:
		ESP_LOGI(TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%"PRIu32" close_by_remote:%d", param->close.status,
			param->close.handle, param->close.async);
		cmdBuf.sppHandle = param->data_ind.handle;
		cmdBuf.command = CMD_BLUETOOTH_CLOSE;
		xQueueSend(xQueueMain, &cmdBuf, 0);

		break;
	case ESP_SPP_START_EVT:
		if (param->start.status == ESP_SPP_SUCCESS) {
			ESP_LOGI(TAG, "ESP_SPP_START_EVT handle:%"PRIu32" sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
				param->start.scn);
			esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
			esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
		} else {
			ESP_LOGE(TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
		}
		break;
	case ESP_SPP_CL_INIT_EVT:
		ESP_LOGI(TAG, "ESP_SPP_CL_INIT_EVT");
		break;
	case ESP_SPP_DATA_IND_EVT:
		/*
		 * We only show the data in which the data length is less than 128 here. If you want to print the data and
		 * the data rate is high, it is strongly recommended to process them in other lower priority application task
		 * rather than in this callback directly. Since the printing takes too much time, it may stuck the Bluetooth
		 * stack and also have a effect on the throughput!
		 */
		ESP_LOGI(TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%"PRIu32, param->data_ind.len, param->data_ind.handle);


		cmdBuf.command = CMD_BLUETOOTH_DATA;
		strcpy((char *)cmdBuf.payload, (char *)param->data_ind.data);
		cmdBuf.payload[param->data_ind.len] = 0;
		cmdBuf.length = param->data_ind.len;
		xQueueSend(xQueueMain, &cmdBuf, 0);

		break;
	case ESP_SPP_CONG_EVT:
		ESP_LOGI(TAG, "ESP_SPP_CONG_EVT");
		break;
	case ESP_SPP_WRITE_EVT:
		ESP_LOGI(TAG, "ESP_SPP_WRITE_EVT");
		break;
	case ESP_SPP_SRV_OPEN_EVT:
		ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%"PRIu32" rem_bda:[%s]", param->srv_open.status,
			param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));

		cmdBuf.sppHandle = param->data_ind.handle;
		cmdBuf.command = CMD_BLUETOOTH_OPEN;
		xQueueSend(xQueueMain, &cmdBuf, 0);

		break;
	case ESP_SPP_SRV_STOP_EVT:
		ESP_LOGI(TAG, "ESP_SPP_SRV_STOP_EVT");
		break;
	case ESP_SPP_UNINIT_EVT:
		ESP_LOGI(TAG, "ESP_SPP_UNINIT_EVT");
		break;
	default:
		break;
	}
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
	char bda_str[18] = {0};

	switch (event) {
	case ESP_BT_GAP_AUTH_CMPL_EVT:{
		if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
			ESP_LOGI(TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
					 bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
		} else {
			ESP_LOGE(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
		}
		break;
	}
	case ESP_BT_GAP_PIN_REQ_EVT:{
		ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
		if (param->pin_req.min_16_digit) {
			ESP_LOGI(TAG, "Input pin code: 0000 0000 0000 0000");
			esp_bt_pin_code_t pin_code = {0};
			esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
		} else {
			ESP_LOGI(TAG, "Input pin code: 1234");
			esp_bt_pin_code_t pin_code;
			pin_code[0] = '1';
			pin_code[1] = '2';
			pin_code[2] = '3';
			pin_code[3] = '4';
			esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
		}
		break;
	}

#if (CONFIG_BT_SSP_ENABLED == true)
	case ESP_BT_GAP_CFM_REQ_EVT:
		ESP_LOGI(TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value:%"PRIu32, param->cfm_req.num_val);
		esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
		break;
	case ESP_BT_GAP_KEY_NOTIF_EVT:
		ESP_LOGI(TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%"PRIu32, param->key_notif.passkey);
		break;
	case ESP_BT_GAP_KEY_REQ_EVT:
		ESP_LOGI(TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
		break;
#endif

	case ESP_BT_GAP_MODE_CHG_EVT:
		ESP_LOGI(TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode,
				 bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
		break;

	default: {
		ESP_LOGI(TAG, "event: %d", event);
		break;
	}
	}
	return;
}

// Timer call back
#if 0
static void timer_cb(TimerHandle_t xTimer)
{
	CMD_t cmdBuf;
	cmdBuf.command = CMD_TIMER;
	cmdBuf.length = sprintf((char *)cmdBuf.payload, "Hello World %d\n", xTaskGetTickCount());
	xQueueSendFromISR(xQueueMain, &cmdBuf, NULL);
}
#endif

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
		ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), cmdBuf.payload, cmdBuf.length, ESP_LOG_DEBUG);
		int txBytes = uart_write_bytes(UART_NUM_1, cmdBuf.payload, cmdBuf.length);
		ESP_LOGI(pcTaskGetName(NULL), "txBytes=%d", txBytes);
	} // end while

	// Never reach here
	vTaskDelete(NULL);
}

static void uart_rx_task(void* pvParameters)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start using GPIO%d", CONFIG_UART_RX_GPIO);
	CMD_t cmdBuf;
	cmdBuf.command = CMD_UART_DATA;
	while (1) {
		cmdBuf.length = uart_read_bytes(UART_NUM_1, cmdBuf.payload, PAYLOAD_SIZE, 10 / portTICK_PERIOD_MS);
		// There is some rxBuf in rx buffer
		if (cmdBuf.length > 0) {
			ESP_LOGD(pcTaskGetName(NULL), "cmdBuf.length=%d", cmdBuf.length);
			ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), cmdBuf.payload, cmdBuf.length, ESP_LOG_DEBUG);
			xQueueSend(xQueueMain, &cmdBuf, portMAX_DELAY);
			
		} else {
			// There is no data in rx buufer
			//ESP_LOGI(pcTaskGetName(NULL), "Read %d", rxBytes);
		}
	} // end while

	// Never reach here
	vTaskDelete(NULL);
}

static void main_task(void* pvParameters)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start");
	CMD_t cmdBuf;
	bool connected = false;
	uint32_t sppHandle = 0;
	while(1) {
		xQueueReceive(xQueueMain, &cmdBuf, portMAX_DELAY);
		ESP_LOGD(pcTaskGetName(NULL), "cmdBuf.command=%d", cmdBuf.command);
		if (cmdBuf.command == CMD_BLUETOOTH_OPEN) {
			connected = true;
			sppHandle = cmdBuf.sppHandle;
		} else if (cmdBuf.command == CMD_BLUETOOTH_CLOSE) {
			connected = false;
			sppHandle = 0;
		} else if (cmdBuf.command == CMD_TIMER) {
			if (connected) {
				esp_spp_write(sppHandle, cmdBuf.length, cmdBuf.payload);
			}
		} else if (cmdBuf.command == CMD_UART_DATA) {
			if (connected) {
				ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), cmdBuf.payload, cmdBuf.length, ESP_LOG_DEBUG);
				esp_spp_write(sppHandle, cmdBuf.length, cmdBuf.payload);
			}
		} else if (cmdBuf.command == CMD_BLUETOOTH_DATA) {
			ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), cmdBuf.payload, cmdBuf.length, ESP_LOG_DEBUG);
			xQueueSend(xQueueUart, &cmdBuf, portMAX_DELAY);
		}
	}
}

void app_main(void)
{
	char bda_str[18] = {0};
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
		ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
		ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bluedroid_init()) != ESP_OK) {
		ESP_LOGE(TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bluedroid_enable()) != ESP_OK) {
		ESP_LOGE(TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
		ESP_LOGE(TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
		ESP_LOGE(TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
	esp_spp_cfg_t bt_spp_cfg = {
		.mode = esp_spp_mode,
		.enable_l2cap_ertm = true,
		.tx_buffer_size = 0, /* Only used for ESP_SPP_MODE_VFS mode */
	};
	if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK) {
#else
	if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
#endif
	//if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
		ESP_LOGE(TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

#if (CONFIG_BT_SSP_ENABLED == true)
	/* Set default parameters for Secure Simple Pairing */
	esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
	esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
	esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

	/*
	 * Set default parameters for Legacy Pairing
	 * Use variable pin, input pin code when pairing
	 */
	esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
	esp_bt_pin_code_t pin_code;
	esp_bt_gap_set_pin(pin_type, 0, pin_code);

	ESP_LOGI(TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

	/* Create Queue */
	xQueueMain = xQueueCreate( 10, sizeof(CMD_t) );
	configASSERT( xQueueMain );
	xQueueUart = xQueueCreate( 10, sizeof(CMD_t) );
	configASSERT( xQueueUart );

#if 0
	/* create and start timer */
	timerHandle = xTimerCreate("Trigger", 5000/portTICK_PERIOD_MS, pdTRUE, NULL, timer_cb);
	configASSERT( timerHandle );
	if (xTimerStart(timerHandle, 0) != pdPASS) {
		ESP_LOGE(TAG, "Unable to start Timer");
		while(1) { vTaskDelay(1); }
	} else {
		ESP_LOGI(TAG, "Success to start Timer");
	}
#endif

	/* uart initialize */
	uart_init();

	/* Start uart task */
	xTaskCreate(uart_tx_task, "uart_tx", 1024*4, NULL, 2, NULL);
	xTaskCreate(uart_rx_task, "uart_rx", 1024*4, NULL, 2, NULL);
	xTaskCreate(main_task, "main", 1024*4, NULL, 2, NULL);
}
