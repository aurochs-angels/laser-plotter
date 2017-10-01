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
struct command{
	char *code;
	double x;
	double y;
};
command setting(char *input){
	command h;
	h.code = input;
	h.x = NULL;
	h.y = NULL;
}
command instrument(char *input, double degree){
	command i;
	i.code = input;
	i.x = degree;
	i.y = NULL;
}

command movement(char *input, double x, double y){
	command m;
	m.code = input;
	m.x = x;
	m.y = y;
}
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
	if(str[0] == 'M'){

	}
	else if(str[0] == 'G'){
		if(str[1] == '4'){
			first = str.substr(3);
			xCoord = std::string::stod(first, &sz );
			send = instrument('M4', xCoord );

		}

	}
	else{
		//TODO error handling
	}

}
