#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
class Gcodes {

public:
	void GCodeParser(const char* taskname,
			uint16_t stacksize, UBaseType_t priority,
			QueueHandle_t UART_data);
	QueueHandle_t getQueueHandle();

private:
	QueueHandle_t commandQueue;
	QueueHandle_t UART_dataQueue;
};
