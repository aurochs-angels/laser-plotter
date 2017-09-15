/*
 * LimitSwitch.h
 *
 *  Created on: 7.9.2017
 *      Author: Ville
 */

#ifndef LIMITSWITCH_H_
#define LIMITSWITCH_H_

#include "DigitalIoPin.h"
#include "aTask.h"
#include "semphr.h"

class LimitSwitch: private DigitalIoPin, public Task {
public:
	LimitSwitch(const char* taskname, uint16_t stacksize, UBaseType_t priority, int port, int pin, bool invert = 0);
	inline SemaphoreHandle_t getSemaphoreHandle(){
		return go;
	}
private:
	SemaphoreHandle_t go;
	void _task() override;
};

#endif /* LIMITSWITCH_H_ */
