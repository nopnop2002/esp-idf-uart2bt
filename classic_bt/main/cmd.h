typedef enum {SPP_SRV_OPEN_EVT, SPP_DATA_IND_EVT, SPP_CLOSE_EVT, SPP_ERROR_EVT, SPP_UART_EVT} COMMAND;

#define PAYLOAD_SIZE 128

typedef struct {
	uint32_t spp_handle;
	uint16_t spp_event_id;
	size_t length;
	uint8_t payload[PAYLOAD_SIZE];
	TaskHandle_t taskHandle;
} CMD_t;

