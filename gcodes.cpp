/*
 * gcodes.cpp
 *
 *  Created on: Sep 21, 2017
 *      Author: aino
 */
#define _GLIBCXX_USE_C99 1
#include "gcodes.h"

void Gcodes::GCodeParser(const char* taskname, uint16_t stacksize,
		UBaseType_t priority, QueueHandle_t UART_data) {
	UART_dataQueue = UART_data;

}
QueueHandle_t Gcodes::getQueueHandle() {

	return commandQueue;

}

void Gcodes::parse() {
	std::string str;
	std::string first;
	std::string second;

	std::string::size_type test;

	std::size_t position;
	std::size_t end_pos;

	double xCoord;
	double yCoord;
	xQueueReceive(UART_dataQueue, &str, portMAX_DELAY);
	Command send;

	/*************String parsing************/
	if (str.length() > 4) {
		if (str.find('G') != std::string::npos) {
			position = str.find('X');
			end_pos = str.find('Y');
			first = str.substr(position+1, end_pos);

			xCoord = std::stod(first, &test);

			position = end_pos;
			end_pos = str.find('A');
			second = str.substr(position+1, end_pos);
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
	else{
		send = setting(E);
	}
	xQueueSend(UART_dataQueue, (void * ) &send, portMAX_DELAY);

}
