/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "cmd.h"

static const char *TAG = "SPP";

#define SPP_SERVER_NAME "SPP_SERVER"
#define DEVICE_NAME "ESP_SPP_ACCEPTOR"

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

extern QueueHandle_t xQueueSpp;
extern QueueHandle_t xQueueUart;

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
		xQueueSendFromISR(xQueueSpp, &cmdBuf, NULL);
		break;
	case ESP_SPP_START_EVT:
		if (param->start.status == ESP_SPP_SUCCESS) {
			ESP_LOGI(TAG, "ESP_SPP_START_EVT handle:%"PRIu32" sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
				param->start.scn);
			esp_bt_dev_set_device_name(DEVICE_NAME);
			esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
		} else {
			ESP_LOGE(TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
		}
		break;
	case ESP_SPP_CL_INIT_EVT:
		ESP_LOGI(TAG, "ESP_SPP_CL_INIT_EVT");
		break;
	case ESP_SPP_DATA_IND_EVT:
		ESP_LOGI(TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%"PRIu32, param->data_ind.len, param->data_ind.handle);
		esp_log_buffer_hex(__FUNCTION__, param->data_ind.data, param->data_ind.len);
		cmdBuf.command = CMD_BLUETOOTH_DATA;
		cmdBuf.length = param->data_ind.len;
		if (cmdBuf.length > PAYLOAD_SIZE) cmdBuf.length = PAYLOAD_SIZE;
		//strcpy((char *)cmdBuf.payload, (char *)param->data_ind.data);
		memcpy(cmdBuf.payload, (char *)param->data_ind.data, cmdBuf.length);
		xQueueSendFromISR(xQueueSpp, &cmdBuf, NULL);
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
		xQueueSendFromISR(xQueueSpp, &cmdBuf, NULL);
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
	} // end swich
	return;
}

void spp_task(void* pvParameters)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start");

	char bda_str[18] = {0};
	esp_err_t ret;
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
		ESP_LOGE(pcTaskGetName(NULL), "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
		ESP_LOGE(pcTaskGetName(NULL), "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0))
	esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
#if (CONFIG_BT_SSP_ENABLED == true)
	bluedroid_cfg.ssp_en = false;
#endif
	if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
		ESP_LOGE(pcTaskGetName(NULL), "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
		return;
	}
#else
	if ((ret = esp_bluedroid_init()) != ESP_OK) {
		ESP_LOGE(pcTaskGetName(NULL), "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
#endif

	if ((ret = esp_bluedroid_enable()) != ESP_OK) {
		ESP_LOGE(pcTaskGetName(NULL), "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
		ESP_LOGE(pcTaskGetName(NULL), "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
		ESP_LOGE(pcTaskGetName(NULL), "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
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
		ESP_LOGE(pcTaskGetName(NULL), "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
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

	ESP_LOGI(pcTaskGetName(NULL), "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

	CMD_t cmdBuf;
	bool connected = false;
	uint32_t sppHandle = 0;
	while(1) {
		xQueueReceive(xQueueSpp, &cmdBuf, portMAX_DELAY);
		ESP_LOGI(pcTaskGetName(NULL), "cmdBuf.command=%d", cmdBuf.command);
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
			ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), cmdBuf.payload, cmdBuf.length, ESP_LOG_INFO);
			xQueueSend(xQueueUart, &cmdBuf, portMAX_DELAY);
		}
	}

	// Never reach here
	vTaskDelete(NULL);
}
