/*
 * LimitSwitch.cpp
 *
 *  Created on: 7.9.2017
 *      Author: Ville
 */

#include "LimitSwitch.h"

LimitSwitch::LimitSwitch(
		const char* taskname,
		uint16_t stacksize,
		UBaseType_t priority,
		SemaphoreHandle_t goSemaphore,
		int port,
		int pin,
		bool invert)
: DigitalIoPin(
		port,
		pin,
		DigitalIoPin::pullup,
		invert),
  Task(
		  taskname,
		  stacksize,
		  priority){
	go = goSemaphore;
}


void LimitSwitch::_task() {
	while(true){
		if(!read()) xSemaphoreGive(go);
		vTaskDelay(portTICK_PERIOD_MS);
	}
}
