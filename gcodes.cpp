/*
 * gcodes.cpp
 *
 *  Created on: Sep 21, 2017
 *      Author: aino
 */
#include "gcodes.h"
/*
 * Gcodes
 * G28 = return to reference
 * M10 = initialize move to edges, set max coordinates
 * G1 Xint Yint A0 = move to coordinate
 * M1 int = pencil rotate
 * M4 int = laser pulse
 *
 */


void Gcodes::GCodeParser(const char* taskname,
			uint16_t stacksize, UBaseType_t priority,
			QueueHandle_t UART_data){
	this->UART_dataQueue = UART_data;

}
QueueHandle_t Gcodes::getQueueHandle() {

	return this->commandQueue;

}


void Gcodes::parse(){
	std::string str;
	std::string first;
	std::string second;
	std::string::size_type sz;
	double xCoord;
	double yCoord;
	xQueueReceive(UART_dataQueue, &str, portMAX_DELAY);
	command send;

	/*************String parsing************/

}
