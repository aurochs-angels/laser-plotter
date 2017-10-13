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

static bool toggle = false;
static DigitalIoPin debugPin(0, 10, DigitalIoPin::output, false);

/* Call Chip_MRT_Init() before calling this constructor
 * Call NVIC_EnableIRQ(MRT_IRQn) after creating both Stepper objects
 * Use either 0 or 1 for MRT_channel! Stepper uses two channels per stepper.
 * */

Stepper* Stepper::stepperByChannel[2];

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
	targetSpeed = 0;
	currentSpeed = 0;
	stepperByChannel[MRT_channel] = this;
	accelMRT_CH->CTRL |= 1;
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

void Stepper::setSpeed(uint16_t speed){
	targetSpeed = speed;
	accelMRT_CH->INTVAL = Chip_Clock_GetSystemClockRate() / (1000/ACCELERATION_STEP_TIME_MS);
	accelMRT_CH->CTRL |= 1;
}

uint16_t Stepper::getCurrentSpeed() const{
	return currentSpeed;
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
	if(currentSpeed < targetSpeed){
		currentSpeed += ACCELERATION_STEP_SIZE;
	} else if(targetSpeed < currentSpeed){
		currentSpeed -= ACCELERATION_STEP_SIZE;
	} else {
		accelMRT_CH->INTVAL = 0 | (1 << 31);
		accelMRT_CH->CTRL &= ~1;
	}

	uint32_t rate = (MAX_RATE * currentSpeed) / 1000;
	stepControl.setInterval(rate);
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
	setSpeed(targetSpeed);
	vTaskDelay(ACCELERATION_STEP_TIME_MS*portTICK_RATE_MS); // Let speed raise above 0
	stepControl.setStepsToRun(steps);
	stepControl.start();
}

uint32_t Stepper::getStepsRequiredToAccelerate() const {
#define ACCELERATION 	(ACCELERATION_STEP_SIZE / ACCELERATION_STEP_TIME_MS)
	uint32_t result = currentSpeed < targetSpeed ?
			(targetSpeed*targetSpeed - currentSpeed*currentSpeed) / (1000*ACCELERATION) :
			(currentSpeed*currentSpeed - targetSpeed*targetSpeed) / (1000*ACCELERATION);
	return result;
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
	if((currentInterval == 0) && (stepsToRun > 0)){
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
	stepsToRun = 2*steps;
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

void Stepper::zeroSteps() {
	steps = 0;
}
