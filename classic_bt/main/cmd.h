typedef enum {CMD_BLUETOOTH_OPEN, CMD_BLUETOOTH_DATA, CMD_BLUETOOTH_CLOSE, CMD_UART_DATA, CMD_TIMER} COMMAND;

#define PAYLOAD_SIZE 128

typedef struct {
	uint32_t sppHandle;
	uint16_t command;
	size_t length;
	uint8_t payload[PAYLOAD_SIZE];
	TaskHandle_t taskHandle;
} CMD_t;

