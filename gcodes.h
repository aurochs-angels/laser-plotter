#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string>
#include <cstdlib>
#include <cstring>
class Gcodes {

public:
	void GCodeParser(const char* taskname,
			uint16_t stacksize, UBaseType_t priority,
			QueueHandle_t UART_data);
	QueueHandle_t getQueueHandle();
	void parse();
	typedef enum {G28, M1, M4, M10, G1} codes;
	struct command{
		codes inserted;
		double x;
		double y;
	};
	Gcodes::command setting(Gcodes::codes input){
		command h;
		h.inserted = input;
		h.x = NULL;
		h.y = NULL;
	}
	Gcodes::command instrument(codes input, double degree){
		command i;
		i.inserted = input;
		i.x = degree;
		i.y = NULL;
	}

	Gcodes::command movement(codes input, double x, double y){
		command m;
		m.inserted = input;
		m.x = x;
		m.y = y;
	}

private:
	QueueHandle_t commandQueue;
	QueueHandle_t UART_dataQueue;

};
