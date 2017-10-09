/*
 * LimitSwitch.h
 *
 *  Created on: 7.9.2017
 *      Author: Ville
 */

#ifndef LIMITSWITCH_H_
#define LIMITSWITCH_H_

#include "InterruptedInputPin.h"
#include "FreeRTOS.h"
#include "aTask.h"
#include "semphr.h"
#include "event_groups.h"

class LimitSwitch_Base {
public:
	LimitSwitch_Base(int port, int pin, int channel) : pinControl(port, pin, true, true, channel, true, channel){};
protected:
	static EventGroupHandle_t eventGroup;
	InterruptedInputPin pinControl;
};

EventGroupHandle_t LimitSwitch_Base::eventGroup = xEventGroupCreate();

template <int channel>
class LimitSwitch : public LimitSwitch_Base {
public:
	LimitSwitch(int port, int pin);

	inline bool isEventBitSet(){
		return (xEventGroupGetBits(eventGroup) >> channel) & 1;
	}
private:
	inline static LimitSwitch* getLimitSwitch(){
		return thisPtr;
	}
	static LimitSwitch* thisPtr;
	static void IRQHandler(portBASE_TYPE* pxHigherPriorityWoken);
};

#include "LimitSwitch.tcc"
#endif /* LIMITSWITCH_TPP_ */
