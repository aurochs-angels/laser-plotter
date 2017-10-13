/*
 * gcodes.cpp
 *
 *  Created on: Sep 21, 2017
 *      Author: aino
 */
#define _GLIBCXX_USE_C99 1
#include "gcodes.h"
/*
 * Gcodes
 * G28 = return to reference
 * M10 = initialize move to edges, set max coordinates
 * G1 Xdouble Ydouble A0 = move to coordinate
 * M1 double = pencil rotate
 * M4 double = laser pulse
 *
 */

void Gcodes::GCodeParser(const char* taskname, uint16_t stacksize,
		UBaseType_t priority, QueueHandle_t UART_data) {
	this->UART_dataQueue = UART_data;

}
QueueHandle_t Gcodes::getQueueHandle() {

	return this->commandQueue;

}

void Gcodes::parse() {
	std::string str;
	std::string first;
	std::string second;
	char* sz;
	std::string::size_type test;
	std::size_t position;
	std::size_t end_pos;
	const char* f;
	double xCoord;
	double yCoord;
	xQueueReceive(UART_dataQueue, &str, portMAX_DELAY);
	command send;

	/*************String parsing************/
	if (str.length() > 4) {
		if (str.find('G') != std::string::npos) {
			position = str.find('X');
			end_pos = str.find('Y');
			first = str.substr(position, end_pos);

			xCoord = std::stod(first, &test);

			position = end_pos;
			end_pos = str.find('A');
			second = str.substr(position, end_pos);
			yCoord = std::stod(second, &test);

			send = movement(G1, xCoord, yCoord);

		} else {
			first = str.substr(3);
			xCoord = stod(first);
			if (str.find("M1") != std::string::npos) {

				send = instrument(M1, xCoord);
			} else {
				send = instrument(M4, xCoord);
			}
		}

	} else if (str.length() < 4) {
		if (str.find('M') != std::string::npos) {
			send = setting(M10);
		} else {
			send = setting(G28);
		}
	}
	xQueueSend(UART_dataQueue, (void *) &send, portMAX_DELAY );

}
