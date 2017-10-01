#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string>
class Gcodes {

public:
	void GCodeParser(const char* taskname,
			uint16_t stacksize, UBaseType_t priority,
			QueueHandle_t UART_data);
	QueueHandle_t getQueueHandle();
	void parse();
	struct command;

private:
	QueueHandle_t commandQueue;
	QueueHandle_t UART_dataQueue;

};
