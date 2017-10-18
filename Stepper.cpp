/*
 * Stepper.cpp
 *
 *  Created on: 9.10.2017
 *      Author: Ville
 */

#include "Stepper.h"
#include "LimitSwitch.h"

#define DEBUG_TOOLS

#ifdef DEBUG_TOOLS
#include "ITM_write.h"
#include "RunningTime.h"
#include "itoa.h"
#endif

const uint32_t Stepper::MAX_RATE = 4000;
const uint32_t Stepper::MIN_RATE = 1000;
const uint32_t Stepper::ACCELERATION_STEP_SIZE = 8;
const double Stepper::ACCELERATION_STEP_TIME_MS = 1.0;
const uint32_t Stepper::ACCELERATION = (ACCELERATION_STEP_SIZE / (ACCELERATION_STEP_TIME_MS/1000));

Stepper* Stepper::stepperByChannel[2];

/* Call Chip_MRT_Init() before calling this constructor
 * Call NVIC_EnableIRQ(MRT_IRQn) after creating both Stepper objects
 * Use either 0 or 1 for MRT_channel! Stepper uses two channels per stepper. (0 and 2) or (1 and 3)
 * */
Stepper::Stepper(uint8_t stepPort, uint8_t stepPin,
		uint8_t dirPort, uint8_t dirPin,
		uint8_t MRT_channel,
		const LimitSwitch_Base& front, const LimitSwitch_Base& back) :
  accelMRT_CH(LPC_MRT_CH(MRT_channel+2)),
  dirControl(dirPort, dirPin, DigitalIoPin::output, false),
  stepControl(stepPort, stepPin, MRT_channel, this),
  limitFront(front), limitBack(back) {
	direction = false;
	stop = false;
	go = xEventGroupCreate();
	steps = 0;
	stepsDone = xSemaphoreCreateBinary();
	targetRate = 0;
	currentRate = 0;
	stepperByChannel[MRT_channel] = this;
}

void Stepper::toggleDirection() {
	direction = !direction;
	dirControl.write(direction);
}

bool Stepper::getDirection() const {
	return direction;
}

void Stepper::setStop(bool stop){
	this->stop = stop;
	accelMRT_CH->INTVAL = 0;
	accelMRT_CH->CTRL &= ~1;
}

void Stepper::setRate(uint32_t rate, bool instant) {
	targetRate = rate;
	if(instant)
		currentRate = rate;
}


uint16_t Stepper::getCurrentRate() const{
	return currentRate;
}

Stepper* Stepper::getStepperByChannel(uint8_t channel) {
	return stepperByChannel[channel];
}

Stepper::StepControl* Stepper::getStepControl() {
	return &stepControl;
}

void Stepper::MRT_callback() {
	_accelerate();
}

void Stepper::_accelerate(){
	if(currentRate < targetRate){
		currentRate += ACCELERATION_STEP_SIZE;
	} else if(targetRate < currentRate){
		currentRate -= ACCELERATION_STEP_SIZE;
	} else {
		accelMRT_CH->INTVAL = 0 | (1 << 31);
		accelMRT_CH->CTRL &= ~1;
	}

	stepControl.setInterval(currentRate);
}

SemaphoreHandle_t Stepper::getStepsDoneSemaphore() {
	return stepsDone;
}

void Stepper::setDirection(bool dir) {
	direction = dir;
	dirControl.write(dir);
}

uint32_t Stepper::getSteps() const {
	return steps;
}


void Stepper::runForSteps(uint32_t steps){
	accelMRT_CH->INTVAL = Chip_Clock_GetSystemClockRate() / (1000/ACCELERATION_STEP_TIME_MS);
	accelMRT_CH->CTRL |= 1;
	stepControl.setStepsToRun(steps);
	stepControl.start();
}

uint32_t Stepper::getStepsRequiredToAccelerate() const {
	uint32_t result = currentRate < targetRate ?
			(targetRate*targetRate - currentRate*currentRate) / (ACCELERATION*2) + 0.5: // +0.5 is for rounding
			(currentRate*currentRate - targetRate*targetRate) / (ACCELERATION*2) + 0.5;
	return result;
}

uint32_t Stepper::getRateAchievable(uint32_t steps, bool max) {
	uint32_t result = max ?
			sqrt(2*ACCELERATION*steps - currentRate*currentRate) + 0.5:
			sqrt(-2*ACCELERATION*steps + currentRate*currentRate) + 0.5;
	return result;
}

void Stepper::zeroSteps() {
	steps = 0;
}

Stepper::StepControl::StepControl(uint8_t stepPort, uint8_t stepPin, uint8_t MRT_channel, Stepper* stepper)
: _stepper(stepper), pin(stepPort, stepPin, DigitalIoPin::output, false) {
	stepCtrlMRT_CH = LPC_MRT_CH(MRT_channel);
}


void Stepper::StepControl::pulse(){
	pin.write(_pulse);
	_pulse = !_pulse;
}

void Stepper::StepControl::setInterval(uint32_t rate){
	if(rate > 0)
		currentInterval = Chip_Clock_GetSystemClockRate() / rate / 2; // Two interrupts = one step
	else
		currentInterval = 0;
}

void Stepper::StepControl::start(){
	stepCtrlMRT_CH->INTVAL = currentInterval;

	if( currentInterval+_stepper->targetRate == 0 ){
		xSemaphoreGive(_stepper->stepsDone);
		stop();
		return;
	}
	stepCtrlMRT_CH->CTRL |= 1; // Enable interrupt
}

void Stepper::StepControl::stop() {
	stepCtrlMRT_CH->INTVAL = 0 | (1 << 31); // Stop immediatly
	stepCtrlMRT_CH->CTRL &= ~1; // Disable interrupt
}

void Stepper::StepControl::setStepsToRun(uint32_t steps) {
	if(steps > 0)
		stepsToRun = (2*steps) - 1;
	else
		stepsToRun = 0;
}

void Stepper::StepControl::MRT_callback(){
	if(stepsToRun != 0){
		bool limit = _stepper->direction ? _stepper->limitBack.isEventBitSet() : _stepper->limitFront.isEventBitSet();

		if(!(limit || _stepper->stop))
		{
			pulse();
			start();
			--stepsToRun;
		} else {
			stop();
			_stepper->stop = true;
		}
	} else {
		stop();
		xSemaphoreGive(_stepper->stepsDone);
	}
	_stepper->steps = !_stepper->direction ? _stepper->steps+1 : _stepper->steps-1;
}

extern "C" void MRT_IRQHandler(void) {
	static bool toggle = false;
	static DigitalIoPin debugPin(0, 0, DigitalIoPin::output, false);
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
	uint32_t interruptPending = Chip_MRT_GetIntPending();
	Chip_MRT_ClearIntPending(interruptPending);
	if(interruptPending & MRTn_INTFLAG(0)){
		Stepper::StepControl* stepControl = Stepper::getStepperByChannel(0)->getStepControl();
		stepControl->MRT_callback();
	}

	if(interruptPending & MRTn_INTFLAG(1)){
		Stepper::StepControl* stepControl = Stepper::getStepperByChannel(1)->getStepControl();
		stepControl->MRT_callback();
	}

	if(interruptPending & MRTn_INTFLAG(2)){
		debugPin.write(toggle = !toggle);

		Stepper* stepper = Stepper::getStepperByChannel(0);
		stepper->MRT_callback();
	}

	if(interruptPending & MRTn_INTFLAG(3)){
		Stepper* stepper = Stepper::getStepperByChannel(1);
		stepper->MRT_callback();
	}
	portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);
}
