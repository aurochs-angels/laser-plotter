/*
 * ButtonEvent.h
 *
 *  Created on: 7.12.2016
 *      Author: Ville
 */

#ifndef INTERRUPTEDINPUTPIN_H_
#define INTERRUPTEDINPUTPIN_H_

#include <cstdint>
#include "board.h"

#define FREERTOS	1

#ifdef FREERTOS
#include "FreeRTOS.h"
#endif

class InterruptedInputPin {
public:
#ifdef FREERTOS
	typedef void (*interruptFunction)(portBASE_TYPE* pxHigherPriorityTaskWoken);
#else
	typedef void (*interruptFunction)();
#endif

	InterruptedInputPin(int port, int pin, bool pullup = true,
						bool risingEdgeInterrupt_ = false, int risingEdgeInterruptIndex_ = 0,
						bool fallingEdgeInterrupt_ = false, int fallingEdgeInterruptIndex_ = 0);
	~InterruptedInputPin();
	bool hasFallingEdgeInterruptEnabled() const;
	int getFallingEdgeInterruptChannel() const;
	int getPin() const;
	int getPort() const;
	bool hasRisingEdgeInterruptEnabled() const;
	int getRisingEdgeInterruptChannel() const;
	void setRisingEdgeInterrupt(bool set);
	void setFallingEdgeInterrupt(bool set);
	static void setInterruptHandler(uint8_t channel, interruptFunction f);
	static interruptFunction getInterruptHandler(uint8_t channel);
private:
	bool risingEdgeInterrupt;
	bool fallingEdgeInterrupt;
	int port;
	int pin;
	int risingEdgeInterruptChannel;
	int fallingEdgeInterruptChannel;
	static interruptFunction handlers[8];
};


#endif /* INTERRUPTEDINPUTPIN_H_ */
