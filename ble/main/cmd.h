#include "esp_gatts_api.h"

typedef enum {CMD_BLUETOOTH_CONNECT, CMD_BLUETOOTH_AUTH, CMD_BLUETOOTH_DATA, CMD_BLUETOOTH_DISCONNECT, CMD_UART_DATA, CMD_TIMER} COMMAND;

#define PAYLOAD_SIZE 128

typedef struct {
	uint16_t spp_conn_id;
	esp_gatt_if_t spp_gatts_if;
	uint16_t command;
	size_t length;
	uint8_t payload[PAYLOAD_SIZE];
	TaskHandle_t taskHandle;
} CMD_t;
