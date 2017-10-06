/*
 * SerialReader.cpp
 *
 *  Created on: Oct 5, 2017
 *      Author: aino
 */

#include "SerialReader.h"
SerialReader::SerialReader(const char* taskname, uint16_t stacksize,
		UBaseType_t priority) {
	char code[32];
	UART_data = xQueueCreate(10, sizeof(code));
}
void SerialReader::receive() {
	char code[32];
	std::string toSend;
	uint8_t len;
	len = USB_receive((uint8_t *) code, (uint32_t) 64);
	toSend = static_cast<std::string>(code);
	xQueueSend(UART_data, static_cast<void *>(&toSend), portMAX_DELAY);
}
QueueHandle_t SerialReader::getQueueHandle() {
	return this->UART_data;
}

