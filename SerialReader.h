/*
 * SerialReader.h
 *
 *  Created on: Oct 5, 2017
 *      Author: aino
 */

#ifndef SERIALREADER_H_
#define SERIALREADER_H_
#include "FreeRTOS.h"
#include "queue.h"
#include <mutex>
#include "Fmutex.h"
#include "user_vcom.h"

class SerialReader {
public:
	SerialReader(const char* taskname,
	uint16_t stacksize, UBaseType_t priority);
	QueueHandle_t getQueueHandle();
	void receive();
private:
	QueueHandle_t UART_data;
};

#endif /* SERIALREADER_H_ */
