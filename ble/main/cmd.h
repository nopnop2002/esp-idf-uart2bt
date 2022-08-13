#include "esp_gatts_api.h"

#define CMD_BLUETOOTH_CONNECT 100
#define CMD_BLUETOOTH_AUTH 200
#define CMD_BLUETOOTH_DATA 300
#define CMD_BLUETOOTH_DISCONNECT 400
#define CMD_UART_DATA 700
#define CMD_TIMER 900

#define PAYLOAD_SIZE 128

typedef struct {
	uint16_t spp_conn_id;
	esp_gatt_if_t spp_gatts_if;
	uint16_t command;
	size_t length;
	uint8_t payload[PAYLOAD_SIZE];
	TaskHandle_t taskHandle;
} CMD_t;
