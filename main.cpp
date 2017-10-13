/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "LimitSwitch.h"

#include "PWMController.h"
#include "Stepper.h"

#include "ITM_write.h"
#include "RunningTime.h"

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, false);
	Chip_PININT_Init(LPC_GPIO_PIN_INT);
	Chip_SWM_Init();
	Chip_MRT_Init();
	ITM_init();
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* the following is required if runtime statistics are to be collected */
extern "C" {

void vConfigureTimerForRunTimeStats( void ) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}

void PWMTest(void* pPWM){
	PWMController& pwm = *static_cast<PWMController*>(pPWM);
	bool rising = true;
	double cycle = 50.0;
	while(true){
		if(rising){
			if(cycle < 100){
				++cycle;
			} else {
				rising = false;
			}
		} else {
			if(cycle > 0){
				--cycle;
			} else {
				rising = true;
			}
		}
		pwm.setDutycycleL(cycle);
		vTaskDelay(20);
	}
}

void StepperTest(void* pStepper){
	Stepper& stepper = *static_cast<Stepper*>(pStepper);
	SemaphoreHandle_t done = stepper.getStepsDoneSemaphore();
	bool direction = true;
	uint16_t speed[3] = {0,0,0};
	uint16_t steps[3] = {0,0,0};
	uint16_t stepsReq[3] = {0,0,0};
	uint32_t time;
	while(true){
		direction = !direction;
		speed[0] = stepper.getCurrentSpeed();
		steps[0] = stepper.getSteps();
		stepper.setDirection(direction);
		RunningTime::start();
		stepper.setSpeed(1000);
		stepsReq[0] = stepper.getStepsRequiredToAccelerate();
		stepper.runForSteps(stepsReq[0]);
		xSemaphoreTake(done, portMAX_DELAY);
//		debugPin.write(toggle = !toggle);

		RunningTime::stop();
		time = RunningTime::getTime();

		speed[1] = stepper.getCurrentSpeed();
		steps[1] = stepper.getSteps();
		RunningTime::start();
		stepper.setSpeed(0);
		stepsReq[1] = stepper.getStepsRequiredToAccelerate();
		stepper.runForSteps(stepsReq[1]);
		xSemaphoreTake(done, portMAX_DELAY);
//		debugPin.write(toggle = !toggle);
		RunningTime::stop();
		time = RunningTime::getTime();
		speed[2] = stepper.getCurrentSpeed();
		steps[2] = stepper.getSteps();
	}
}

int main(void)
{
	prvSetupHardware();
	RunningTime::setup();


	PWMController* pwm = new PWMController(LPC_SCT0);
	pwm->initCounterL(10000, 50, true, 1);
	pwm->setOutputL(1, 1, 0, true);
	pwm->startCounterL();

	LimitSwitch<0>* ls1 = new LimitSwitch<0>(0, 27);
	LimitSwitch<1>* ls2 = new LimitSwitch<1>(0, 28);
	Stepper* stepper = new Stepper(
			//"Stepper", configMINIMAL_STACK_SIZE*2, // Taskname, stack size
			//(tskIDLE_PRIORITY + 1UL), // Priority
			0, 24, // Motor drive pin&port
			1, 0, // Motor direction pin&port
			0, // MRT channel (0 or 1)
			*ls2, *ls1); // LimitSwitch_Base& front / back
	NVIC_EnableIRQ(MRT_IRQn);

	xTaskCreate(PWMTest, "PWM", configMINIMAL_STACK_SIZE*2, pwm, (tskIDLE_PRIORITY + 1UL), nullptr);
	xTaskCreate(StepperTest, "stepperTest", configMINIMAL_STACK_SIZE*2, stepper, (tskIDLE_PRIORITY + 1UL), nullptr);

	vTaskStartScheduler();

	return 1;
}

