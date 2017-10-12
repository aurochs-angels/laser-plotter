/*
 * PWMController.h
 *
 *  Created on: 5.10.2017
 *      Author: Ville
 */

#ifndef PWMCONTROLLER_H_
#define PWMCONTROLLER_H_

#include "chip.h"
#include <cstdint>

class PWMController {
struct Counter {
	uint16_t freq;
	double dutycycle;
	bool bidirectional;
};
public:
	PWMController(LPC_SCT_T* pSCTimer);

	void initCounterL(uint16_t freq, double dutycycle, bool bidirectional);
	void initCounterH(uint16_t freq, double dutycycle, bool bidirectional);

	void startCounterL();
	void startCounterH();
	void stopCounterL();
	void stopCounterH();

	void setDutycycleL(double dutycycle);
	void setDutycycleH(double dutycycle);
	double getDutycycleL();
	double getDutycycleH();

	void setFrequencyL(uint16_t freq);
	void setFrequencyH(uint16_t freq);
	uint16_t getFrequencyL();
	uint16_t getFrequencyH();

	uint16_t periodFromFrequency(uint16_t freq, bool bidirectional);

	void setOutputL(uint8_t port, uint8_t pin, uint8_t outindex, bool on);
	void setOutputH(uint8_t port, uint8_t pin, uint8_t outindex, bool on);
private:
	LPC_SCT_T* timer;
	Counter L;
	Counter H;

	CHIP_SWM_PIN_FIXED_T getFixedSWMPin(uint8_t outindex);
	CHIP_SWM_PIN_MOVABLE_T getMovableSWMPin(uint8_t outindex);
};

#endif /* PWMCONTROLLER_H_ */
