#define CMD_BLUETOOTH_OPEN 100
#define CMD_BLUETOOTH_DATA 200
#define CMD_BLUETOOTH_CLOSE 300
#define CMD_UART_DATA 400
#define CMD_TIMER 900

#define PAYLOAD_SIZE 128

typedef struct {
	uint32_t sppHandle;
	uint16_t command;
	size_t length;
	uint8_t payload[PAYLOAD_SIZE];
	TaskHandle_t taskHandle;
} CMD_t;

