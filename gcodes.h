#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string>
#include <cstdlib>
class Gcodes {

public:
	void GCodeParser(const char* taskname,
			uint16_t stacksize, UBaseType_t priority,
			QueueHandle_t UART_data);
	QueueHandle_t getQueueHandle();
	void parse();
	struct command{
		char *code;
		double x;
		double y;
	};
	command setting(char input[32]){
		command h;
		h.code = input;
		h.x = NULL;
		h.y = NULL;
	}
	command instrument(char input[32], double degree){
		command i;
		i.code = input;
		i.x = degree;
		i.y = NULL;
	}

	command movement(char input[32], double x, double y){
		command m;
		m.code = input;
		m.x = x;
		m.y = y;
	}

private:
	QueueHandle_t commandQueue;
	QueueHandle_t UART_dataQueue;

};
