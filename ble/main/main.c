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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/uart.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include "cmd.h"

static const char *TAG = "MAIN";

#define SPP_PROFILE_NUM 1
#define SPP_PROFILE_APP_IDX 0
#define ESP_SPP_APP_ID 0x56
#ifdef CONFIG_IDF_TARGET_ESP32
#define SAMPLE_DEVICE_NAME "ESP32_SPP_SERVER" //The Device Name Characteristics in GAP
#elif defined CONFIG_IDF_TARGET_ESP32S3
#define SAMPLE_DEVICE_NAME "ESP32S3_SPP_SERVER" //The Device Name Characteristics in GAP
#elif defined CONFIG_IDF_TARGET_ESP32C2
#define SAMPLE_DEVICE_NAME "ESP32C2_SPP_SERVER" //The Device Name Characteristics in GAP
#elif defined CONFIG_IDF_TARGET_ESP32C3
#define SAMPLE_DEVICE_NAME "ESP32C3_SPP_SERVER" //The Device Name Characteristics in GAP
#elif defined CONFIG_IDF_TARGET_ESP32C6
#define SAMPLE_DEVICE_NAME "ESP32C6_SPP_SERVER" //The Device Name Characteristics in GAP
#endif

#define SPP_SVC_INST_ID 0
#define SPP_DATA_MAX_LEN (512)
//#define SPP_CMD_MAX_LEN (20)
//#define SPP_STATUS_MAX_LEN (20)
//#define SPP_DATA_BUFF_MAX_LEN (2*1024)
///Attributes State Machine
enum{
	SPP_IDX_SVC,

	SPP_IDX_SPP_DATA_RECV_CHAR,
	SPP_IDX_SPP_DATA_RECV_VAL,

	SPP_IDX_SPP_DATA_NOTIFY_CHAR,
	SPP_IDX_SPP_DATA_NOTIFY_VAL,
	SPP_IDX_SPP_DATA_NOTIFY_CFG,

#if 0
	SPP_IDX_SPP_COMMAND_CHAR,
	SPP_IDX_SPP_COMMAND_VAL,

	SPP_IDX_SPP_STATUS_CHAR,
	SPP_IDX_SPP_STATUS_VAL,
	SPP_IDX_SPP_STATUS_CFG,

	SPP_IDX_SPP_HEARTBEAT_CHAR,
	SPP_IDX_SPP_HEARTBEAT_VAL,
	SPP_IDX_SPP_HEARTBEAT_CFG,
#endif

	SPP_IDX_NB,
};

/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE 0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY 0xABF2
#if 0
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE 0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY 0xABF4
#define ESP_GATT_UUID_SPP_HEARTBEAT 0xABF5
#endif

static const uint8_t spp_adv_data[23] = {
	/* Flags */
	0x02,0x01,0x06,
	/* Complete List of 16-bit Service Class UUIDs */
	0x03,0x03,0xF0,0xAB,
	/* Complete Local Name in advertising */
	0x0F,0x09, 'E', 'S', 'P', '_', 'S', 'P', 'P', '_', 'S', 'E', 'R','V', 'E', 'R'
};

//static uint16_t spp_conn_id = 0xffff;
//static esp_gatt_if_t spp_gatts_if = 0xff;
//static esp_bd_addr_t spp_remote_bda = {0x0,};

QueueHandle_t xQueueMain = NULL;
QueueHandle_t xQueueUart;

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
	.adv_int_min		= 0x20,
	.adv_int_max		= 0x40,
	.adv_type			= ADV_TYPE_IND,
	.own_addr_type		= BLE_ADDR_TYPE_PUBLIC,
	.channel_map		= ADV_CHNL_ALL,
	.adv_filter_policy	= ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

//#define GATTS_DEMO_CHAR_VAL_LEN_MAX				0x40

#define ADV_CONFIG_FLAG							  (1 << 0)
#define SCAN_RSP_CONFIG_FLAG					  (1 << 1)

static uint8_t adv_config_done = 0;

static uint8_t test_manufacturer[3]={'E', 'S', 'P'};

static uint8_t sec_service_uuid[16] = {
	/* LSB <--------------------------------------------------------------------------------> MSB */
	//first uuid, 16bit, [12],[13] is the value
	0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x18, 0x0D, 0x00, 0x00,
};

// config adv data
static esp_ble_adv_data_t spp_adv_config = {
	.set_scan_rsp = false,
	.include_txpower = true,
	.min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
	.max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
	.appearance = 0x00,
	.manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
	.p_manufacturer_data =	NULL, //&test_manufacturer[0],
	.service_data_len = 0,
	.p_service_data = NULL,
	.service_uuid_len = sizeof(sec_service_uuid),
	.p_service_uuid = sec_service_uuid,
	.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// config scan response data
static esp_ble_adv_data_t spp_scan_rsp_config = {
	.set_scan_rsp = true,
	.include_name = true,
	.manufacturer_len = sizeof(test_manufacturer),
	.p_manufacturer_data = test_manufacturer,
};

struct gatts_profile_inst {
	esp_gatts_cb_t gatts_cb;
	uint16_t gatts_if;
	uint16_t app_id;
	uint16_t conn_id;
	uint16_t service_handle;
	esp_gatt_srvc_id_t service_id;
	uint16_t char_handle;
	esp_bt_uuid_t char_uuid;
	esp_gatt_perm_t perm;
	esp_gatt_char_prop_t property;
	uint16_t descr_handle;
	esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
										esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
	[SPP_PROFILE_APP_IDX] = {
		.gatts_cb = gatts_profile_event_handler,
		.gatts_if = ESP_GATT_IF_NONE,		/* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
	},
};

/*
 *	SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE	(sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;

#if 0
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
#endif

///SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

#if 0
///SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t  spp_command_val[10] = {0x00};
#endif

#if 0
///SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t  spp_status_val[10] = {0x00};
static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};
#endif

#if 0
///SPP Server - Heart beat characteristic, notify&write&read
static const uint16_t spp_heart_beat_uuid = ESP_GATT_UUID_SPP_HEARTBEAT;
static const uint8_t  spp_heart_beat_val[2] = {0x00, 0x00};
static const uint8_t  spp_heart_beat_ccc[2] = {0x00, 0x00};
#endif

/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
	//SPP -  Service Declaration
	[SPP_IDX_SVC]						=
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
	sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

	//SPP -  data receive characteristic Declaration
	[SPP_IDX_SPP_DATA_RECV_CHAR]			=
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

	//SPP -  data receive characteristic Value
	[SPP_IDX_SPP_DATA_RECV_VAL]					=
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
	SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

	//SPP -  data notify characteristic Declaration
	[SPP_IDX_SPP_DATA_NOTIFY_CHAR]	=
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

	//SPP -  data notify characteristic Value
	[SPP_IDX_SPP_DATA_NOTIFY_VAL]	=
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
	SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

	//SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
	[SPP_IDX_SPP_DATA_NOTIFY_CFG]		  =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
	sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

#if 0
	//SPP -  command characteristic Declaration
	[SPP_IDX_SPP_COMMAND_CHAR]			  =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

	//SPP -  command characteristic Value
	[SPP_IDX_SPP_COMMAND_VAL]				  =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
	SPP_CMD_MAX_LEN,sizeof(spp_command_val), (uint8_t *)spp_command_val}},

	//SPP -  status characteristic Declaration
	[SPP_IDX_SPP_STATUS_CHAR]			 =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

	//SPP -  status characteristic Value
	[SPP_IDX_SPP_STATUS_VAL]				 =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
	SPP_STATUS_MAX_LEN,sizeof(spp_status_val), (uint8_t *)spp_status_val}},

	//SPP -  status characteristic - Client Characteristic Configuration Descriptor
	[SPP_IDX_SPP_STATUS_CFG]		 =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
	sizeof(uint16_t),sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},

	//SPP -  Heart beat characteristic Declaration
	[SPP_IDX_SPP_HEARTBEAT_CHAR]  =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

	//SPP -  Heart beat characteristic Value
	[SPP_IDX_SPP_HEARTBEAT_VAL]   =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_heart_beat_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
	sizeof(spp_heart_beat_val), sizeof(spp_heart_beat_val), (uint8_t *)spp_heart_beat_val}},

	//SPP -  Heart beat characteristic - Client Characteristic Configuration Descriptor
	[SPP_IDX_SPP_HEARTBEAT_CFG]			=
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
	//sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_heart_beat_ccc}},
	sizeof(uint16_t),sizeof(spp_heart_beat_ccc), (uint8_t *)spp_heart_beat_ccc}},
#endif
};

static char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
   char *key_str = NULL;
   switch(key_type) {
	case ESP_LE_KEY_NONE:
		key_str = "ESP_LE_KEY_NONE";
		break;
	case ESP_LE_KEY_PENC:
		key_str = "ESP_LE_KEY_PENC";
		break;
	case ESP_LE_KEY_PID:
		key_str = "ESP_LE_KEY_PID";
		break;
	case ESP_LE_KEY_PCSRK:
		key_str = "ESP_LE_KEY_PCSRK";
		break;
	case ESP_LE_KEY_PLK:
		key_str = "ESP_LE_KEY_PLK";
		break;
	case ESP_LE_KEY_LLK:
		key_str = "ESP_LE_KEY_LLK";
		break;
	case ESP_LE_KEY_LENC:
		key_str = "ESP_LE_KEY_LENC";
		break;
	case ESP_LE_KEY_LID:
		key_str = "ESP_LE_KEY_LID";
		break;
	case ESP_LE_KEY_LCSRK:
		key_str = "ESP_LE_KEY_LCSRK";
		break;
	default:
		key_str = "INVALID BLE KEY TYPE";
		break;

   }

   return key_str;
}

static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req)
{
   char *auth_str = NULL;
   switch(auth_req) {
	case ESP_LE_AUTH_NO_BOND:
		auth_str = "ESP_LE_AUTH_NO_BOND";
		break;
	case ESP_LE_AUTH_BOND:
		auth_str = "ESP_LE_AUTH_BOND";
		break;
	case ESP_LE_AUTH_REQ_MITM:
		auth_str = "ESP_LE_AUTH_REQ_MITM";
		break;
	case ESP_LE_AUTH_REQ_BOND_MITM:
		auth_str = "ESP_LE_AUTH_REQ_BOND_MITM";
		break;
	case ESP_LE_AUTH_REQ_SC_ONLY:
		auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
		break;
	case ESP_LE_AUTH_REQ_SC_BOND:
		auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
		break;
	case ESP_LE_AUTH_REQ_SC_MITM:
		auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
		break;
	case ESP_LE_AUTH_REQ_SC_MITM_BOND:
		auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
		break;
	default:
		auth_str = "INVALID BLE AUTH REQ";
		break;
   }

   return auth_str;
}

static void show_bonded_devices(void)
{
	int dev_num = esp_ble_get_bond_device_num();

	esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
	esp_ble_get_bond_device_list(&dev_num, dev_list);
	ESP_LOGI(__FUNCTION__, "Bonded devices number : %d", dev_num);
	ESP_LOGI(__FUNCTION__, "Bonded devices list : %d", dev_num);
	for (int i = 0; i < dev_num; i++) {
		esp_log_buffer_hex(__FUNCTION__, (void *)dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
	}

	free(dev_list);
}

static void __attribute__((unused)) remove_all_bonded_devices(void)
{
	int dev_num = esp_ble_get_bond_device_num();

	esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
	esp_ble_get_bond_device_list(&dev_num, dev_list);
	for (int i = 0; i < dev_num; i++) {
		esp_ble_remove_bond_device(dev_list[i].bd_addr);
	}

	free(dev_list);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	ESP_LOGI(__FUNCTION__, "GAP_EVT, event %d", event);
	CMD_t cmdBuf;

	switch (event) {
	case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
		if (adv_config_done == 0){
			esp_ble_gap_start_advertising(&spp_adv_params);
		}
		break;
	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~ADV_CONFIG_FLAG);
		if (adv_config_done == 0){
			esp_ble_gap_start_advertising(&spp_adv_params);
		}
		break;
	case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
		//advertising start complete event to indicate advertising start successfully or failed
		if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(__FUNCTION__, "advertising start failed, error status = %x", param->adv_start_cmpl.status);
			break;
		}
		ESP_LOGI(__FUNCTION__, "advertising start success");
		break;
	case ESP_GAP_BLE_PASSKEY_REQ_EVT:							/* passkey request event */
		ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
		/* Call the following function to input the passkey which is displayed on the remote device */
		//esp_ble_passkey_reply(spp_profile_tab[SPP_PROFILE_APP_IDX].remote_bda, true, 0x00);
		break;
	case ESP_GAP_BLE_OOB_REQ_EVT: {
		ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_OOB_REQ_EVT");
		uint8_t tk[16] = {1}; //If you paired with OOB, both devices need to use the same tk
		esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
		break;
	}
	case ESP_GAP_BLE_LOCAL_IR_EVT:								 /* BLE local IR event */
		ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_LOCAL_IR_EVT");
		break;
	case ESP_GAP_BLE_LOCAL_ER_EVT:								 /* BLE local ER event */
		ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_LOCAL_ER_EVT");
		break;
	case ESP_GAP_BLE_NC_REQ_EVT:
		/* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
		show the passkey number to the user to confirm it with the number displayed by peer device. */
		esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
		ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%"PRIu32, param->ble_security.key_notif.passkey);
		break;
	case ESP_GAP_BLE_SEC_REQ_EVT:
		/* send the positive(true) security response to the peer device to accept the security request.
		If not accept the security request, should send the security response with negative(false) accept value*/
		esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
		break;
	case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
		///show the passkey number to the user to input it in the peer device.
		ESP_LOGI(__FUNCTION__, "The passkey Notify number:%06"PRIu32, param->ble_security.key_notif.passkey);
		break;
	case ESP_GAP_BLE_KEY_EVT:
		//shows the ble key info share with peer device to the user.
		ESP_LOGI(__FUNCTION__, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
		break;
	case ESP_GAP_BLE_AUTH_CMPL_EVT: {
		esp_bd_addr_t bd_addr;
		memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
		ESP_LOGI(__FUNCTION__, "remote BD_ADDR: %08x%04x",\
				(bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
				(bd_addr[4] << 8) + bd_addr[5]);
		ESP_LOGI(__FUNCTION__, "address type = %d", param->ble_security.auth_cmpl.addr_type);
		ESP_LOGI(__FUNCTION__, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
		if(!param->ble_security.auth_cmpl.success) {
			ESP_LOGI(__FUNCTION__, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
		} else {
			ESP_LOGI(__FUNCTION__, "auth mode = %s",esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
		}
		show_bonded_devices();

		cmdBuf.command = CMD_BLUETOOTH_AUTH;
		xQueueSendFromISR(xQueueMain, &cmdBuf, NULL);
		break;
	}
	case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
		ESP_LOGD(__FUNCTION__, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d", param->remove_bond_dev_cmpl.status);
		ESP_LOGI(__FUNCTION__, "ESP_GAP_BLE_REMOVE_BOND_DEV");
		ESP_LOGI(__FUNCTION__, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
		esp_log_buffer_hex(__FUNCTION__, (void *)param->remove_bond_dev_cmpl.bd_addr, sizeof(esp_bd_addr_t));
		ESP_LOGI(__FUNCTION__, "------------------------------------");
		break;
	}
	case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
		if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS){
			ESP_LOGE(__FUNCTION__, "config local privacy failed, error status = %x", param->local_privacy_cmpl.status);
			break;
		}

		esp_err_t ret = esp_ble_gap_config_adv_data(&spp_adv_config);
		if (ret){
			ESP_LOGE(__FUNCTION__, "config adv data failed, error code = %x", ret);
		}else{
			adv_config_done |= ADV_CONFIG_FLAG;
		}

		ret = esp_ble_gap_config_adv_data(&spp_scan_rsp_config);
		if (ret){
			ESP_LOGE(__FUNCTION__, "config adv data failed, error code = %x", ret);
		}else{
			adv_config_done |= SCAN_RSP_CONFIG_FLAG;
		}

		break;
	default:
		break;
	}
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
										esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
	CMD_t cmdBuf;

	ESP_LOGV(__FUNCTION__, "event = %x\n",event);
	switch (event) {
		case ESP_GATTS_REG_EVT:
			esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
			//generate a resolvable random address
			esp_ble_gap_config_local_privacy(true);
			esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));
			esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
			break;
		case ESP_GATTS_READ_EVT:
			break;
		case ESP_GATTS_WRITE_EVT:
			ESP_LOGI(__FUNCTION__, "ESP_GATTS_WRITE_EVT");
			esp_log_buffer_hex(__FUNCTION__, param->write.value, param->write.len);
			cmdBuf.command = CMD_BLUETOOTH_DATA;
			cmdBuf.length = param->write.len;
			memcpy(cmdBuf.payload, (char *)param->write.value, param->write.len);
			xQueueSendFromISR(xQueueMain, &cmdBuf, NULL);
			break;
		case ESP_GATTS_EXEC_WRITE_EVT:
			break;
		case ESP_GATTS_MTU_EVT:
			break;
		case ESP_GATTS_CONF_EVT:
			break;
		case ESP_GATTS_UNREG_EVT:
			break;
		case ESP_GATTS_DELETE_EVT:
			break;
		case ESP_GATTS_START_EVT:
			break;
		case ESP_GATTS_STOP_EVT:
			break;
		case ESP_GATTS_CONNECT_EVT:
			ESP_LOGI(__FUNCTION__, "ESP_GATTS_CONNECT_EVT");
			/* start security connect with peer device when receive the connect event sent by the master */
			esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
			cmdBuf.command = CMD_BLUETOOTH_CONNECT;
			cmdBuf.spp_conn_id = p_data->connect.conn_id;
			cmdBuf.spp_gatts_if = gatts_if;
			xQueueSendFromISR(xQueueMain, &cmdBuf, NULL);
			break;
		case ESP_GATTS_DISCONNECT_EVT:
			ESP_LOGI(__FUNCTION__, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
			cmdBuf.command = CMD_BLUETOOTH_DISCONNECT;
			xQueueSendFromISR(xQueueMain, &cmdBuf, NULL);
			/* start advertising again when missing the connect */
			esp_ble_gap_start_advertising(&spp_adv_params);
			break;
		case ESP_GATTS_OPEN_EVT:
			break;
		case ESP_GATTS_CANCEL_OPEN_EVT:
			break;
		case ESP_GATTS_CLOSE_EVT:
			break;
		case ESP_GATTS_LISTEN_EVT:
			break;
		case ESP_GATTS_CONGEST_EVT:
			break;
		case ESP_GATTS_CREAT_ATTR_TAB_EVT:
			ESP_LOGI(__FUNCTION__, "The number handle =%x",param->add_attr_tab.num_handle);
			if (param->add_attr_tab.status != ESP_GATT_OK){
				ESP_LOGE(__FUNCTION__, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
			}
			else if (param->add_attr_tab.num_handle != SPP_IDX_NB){
				ESP_LOGE(__FUNCTION__, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
			}
			else {
				memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
				esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
			}
			break;
		default:
		   break;
	}
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
								esp_ble_gatts_cb_param_t *param)
{
	/* If event is register event, store the gatts_if for each profile */
	if (event == ESP_GATTS_REG_EVT) {
		if (param->reg.status == ESP_GATT_OK) {
			spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
		} else {
			ESP_LOGI(__FUNCTION__, "Reg app failed, app_id %04x, status %d\n",
					param->reg.app_id,
					param->reg.status);
			return;
		}
	}

	do {
		int idx;
		for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
			if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
					gatts_if == spp_profile_tab[idx].gatts_if) {
				if (spp_profile_tab[idx].gatts_cb) {
					spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
				}
			}
		}
	} while (0);
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
		ESP_LOGD(pcTaskGetName(NULL), "cmdBuf.length=%d", cmdBuf.length);
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
			ESP_LOGI(pcTaskGetName(NULL), "cmdBuf.length=%d", cmdBuf.length);
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

void main_task(void * arg)
{
	ESP_LOGI(pcTaskGetName(0), "Start");
	CMD_t cmdBuf;
	uint16_t spp_conn_id = 0xffff;
	esp_gatt_if_t spp_gatts_if = 0xff;
	bool connected = false;

	while(1){
		vTaskDelay(50 / portTICK_PERIOD_MS);
		xQueueReceive(xQueueMain, &cmdBuf, portMAX_DELAY);
		ESP_LOGD(pcTaskGetName(NULL), "cmdBuf.command=%d connected=%d", cmdBuf.command, connected);
		if (cmdBuf.command == CMD_BLUETOOTH_CONNECT) {
			spp_conn_id = cmdBuf.spp_conn_id;
			spp_gatts_if = cmdBuf.spp_gatts_if;
		} else if (cmdBuf.command == CMD_BLUETOOTH_AUTH) {
			connected = true;
		} else if (cmdBuf.command == CMD_BLUETOOTH_DISCONNECT) {
			connected = false;
		} else if (cmdBuf.command == CMD_TIMER) {
			if (connected) {
				uint8_t	value[64];
				sprintf((char *)value, "EspressIF\n");
				uint16_t value_len = strlen((char *)value);
				ESP_LOGI(pcTaskGetName(0), "value_len=%d", value_len);
				esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NOTIFY_VAL], value_len, value, false);
			}
		} else if (cmdBuf.command == CMD_UART_DATA) {
			if (connected) {
				ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), cmdBuf.payload, cmdBuf.length, ESP_LOG_DEBUG);
				esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NOTIFY_VAL], cmdBuf.length, cmdBuf.payload, false);
			}
		} else if (cmdBuf.command == CMD_BLUETOOTH_DATA) {
			ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), cmdBuf.payload, cmdBuf.length, ESP_LOG_DEBUG);
			xQueueSend(xQueueUart, &cmdBuf, portMAX_DELAY);
		}
	} // end while

	// never reach here
	vTaskDelete(NULL);
}

void app_main(void)
{
	// Initialize NVS.
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
		ESP_LOGE(TAG, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
		ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0))
	esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
	if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
		ESP_LOGE(TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
		return;
	}
#else
	if ((ret = esp_bluedroid_init()) != ESP_OK) {
		ESP_LOGE(TAG, "%s init bluedroid failed: %s", __func__, esp_err_to_name(ret));
		return;
	}
#endif

	if ((ret = esp_bluedroid_enable()) != ESP_OK) {
		ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_ble_gatts_register_callback(gatts_event_handler)) != ESP_OK) {
		ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
		return;
	}

	if ((ret = esp_ble_gap_register_callback(gap_event_handler)) != ESP_OK) {
		ESP_LOGE(TAG, "gap register error, error code = %x", ret);
		return;
	}

	if ((ret = esp_ble_gatts_app_register(ESP_SPP_APP_ID)) != ESP_OK) {
		ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
		return;
	}

	/* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
	esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;		//bonding with peer device after authentication
	esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;			//set the IO capability to No output No input
	uint8_t key_size = 16;		//the key size should be 7~16 bytes
	uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	//set static passkey
	uint32_t passkey = 123456;
	uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
	uint8_t oob_support = ESP_BLE_OOB_DISABLE;
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
	/* If your BLE device acts as a Slave, the init_key means you hope which types of key of the master should distribute to you,
	and the response key means which key you can distribute to the master;
	If your BLE device acts as a master, the response key means you hope which types of key of the slave should distribute to you,
	and the init key means which key you can distribute to the slave. */
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

	/* Just show how to clear all the bonded devices
	 * Delay 30s, clear all the bonded devices
	 *
	 * vTaskDelay(30000 / portTICK_PERIOD_MS);
	 * remove_all_bonded_devices();
	 */

#if 0
	/* create and start timer */
	TimerHandle_t timerHandle = xTimerCreate("Trigger", 5000/portTICK_PERIOD_MS, pdTRUE, NULL, timer_cb);
	configASSERT( timerHandle );
	if (xTimerStart(timerHandle, 0) != pdPASS) {
		ESP_LOGE(TAG, "Unable to start Timer");
		while(1) { vTaskDelay(1); }
	} else {
		ESP_LOGI(TAG, "Success to start Timer");
	}
#endif

	/* Create Queue */
	xQueueMain = xQueueCreate(10, sizeof(CMD_t));
	configASSERT( xQueueMain );
	xQueueUart = xQueueCreate( 10, sizeof(CMD_t) );
	configASSERT( xQueueUart );

	/* uart initialize */
	uart_init();

	/* Start tasks */
	xTaskCreate(uart_tx_task, "uart_tx", 1024*4, NULL, 2, NULL);
	xTaskCreate(uart_rx_task, "uart_rx", 1024*4, NULL, 2, NULL);
	xTaskCreate(main_task, "main", 1024*4, NULL, 10, NULL);
}
