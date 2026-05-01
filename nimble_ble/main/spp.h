typedef enum {SPP_CONNECT_EVT, SPP_AUTH_EVT, SPP_WRITE_EVT, SPP_DISCONNECT_EVT, SPP_UART_EVT} COMMAND;

#define PAYLOAD_SIZE 128

typedef struct {
	//uint16_t spp_conn_id;
	//esp_gatt_if_t spp_gatts_if;
	uint16_t spp_event_id;
	size_t length;
	uint8_t payload[PAYLOAD_SIZE];
	TaskHandle_t taskHandle;
} CMD_t;
