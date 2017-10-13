#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string>
#include <cstdlib>
#include <cstring>
/*
 * Gcodes
 * G28 = return to reference
 * M10 = initialize move to edges, set max coordinates
 * G1 Xdouble Ydouble A0 = move to coordinate
 * M1 double = pencil rotate
 * M4 double = laser pulse
 *
 */
enum Codes {G28, M1, M4, M10, G1, E} ;
	struct Command{
		Codes inserted;
		double x;
		double y;
	};
	Command setting(Codes input){
		Command h;
		h.inserted = input;

		return h;
	}
	Command instrument(Codes input, double degree){
		Command i;
		i.inserted = input;
		i.x = degree;
		return i;
	}

	Command movement(Codes input, double x, double y){
		Command m;
		m.inserted = input;
		m.x = x;
		m.y = y;
		return m;
	}

class Gcodes {

public:
	void GCodeParser(const char* taskname,
			uint16_t stacksize, UBaseType_t priority,
			QueueHandle_t UART_data);
	QueueHandle_t getQueueHandle();
	void parse();

private:
	QueueHandle_t commandQueue;
	QueueHandle_t UART_dataQueue;

};
