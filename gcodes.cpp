/*
 * gcodes.cpp
 *
 *  Created on: Sep 21, 2017
 *      Author: aino
 */
#include "gcodes.h"

/*enum gCodes::get() {

	return enum;
};*/

void Gcodes::GCodeParser(const char* taskname,
			uint16_t stacksize, UBaseType_t priority,
			QueueHandle_t UART_data){
	this.UART_dataQueue = UART_data;

}
QueueHandle_t Gcodes::getQueueHandle() {

	return this.commandQueue;

}


