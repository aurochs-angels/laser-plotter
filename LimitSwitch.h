/*
 * LimitSwitch.h
 *
 *  Created on: 7.9.2017
 *      Author: Ville
 */

#ifndef LIMITSWITCH_H_
#define LIMITSWITCH_H_

#include "InterruptedInputPin.h"
#include "FreeRTOS/FreeRTOS.h"
#include "aTask.h"
#include "semphr.h"
#include "event_groups.h"
#include "ITM_write.h"

/* Anything non-templated should be here */
class LimitSwitch_Base {
public:
	LimitSwitch_Base(int port, int pin, int channel);
	inline static EventGroupHandle_t getEventGroup(){
		return eventGroup;
	}
	inline bool isEventBitSet() const {
		return (xEventGroupGetBits(eventGroup) >> _channel) & 1;
	}
protected:
	static EventGroupHandle_t eventGroup;
	InterruptedInputPin pinControl;
	int _channel;
};

/* Only one limit switch per channel */
template <int channel>
class LimitSwitch : public LimitSwitch_Base {
public:
	LimitSwitch(int port, int pin);
private:
	inline static LimitSwitch* getLimitSwitch(){
		return thisPtr;
	}
	static LimitSwitch* thisPtr;
	static void IRQHandler(portBASE_TYPE* pxHigherPriorityWoken);
};

#include "LimitSwitch.tcc"
#endif /* LIMITSWITCH_H_ */
