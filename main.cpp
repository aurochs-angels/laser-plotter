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
	LimitSwitch<0> lsY1(0, 29);
	LimitSwitch<1> lsY2(0, 9);
	LimitSwitch<2> lsX1(1, 3);
	LimitSwitch<3> lsX2(0, 0);

	Stepper stepperX(
			0, 27, // Motor drive pin&port
			0, 28, // Motor direction pin&port
			0, // MRT channel (0 or 1)
			lsX2, lsX1); // LimitSwitch_Base& front / back

	Stepper stepperY(
			0, 24,
			1, 0,
			1,
			lsY2, lsY1);

	PWMController pen(LPC_SCT0);
	pen.initCounterL(50, 3, true, 1);
	pen.setOutputL(0, 10, 0, true);
	pen.startCounterL();

	stepperY.calibrate();
	stepperX.calibrate();
	Stepper::waitForAllSteppers();

	pen.setDutycycleL(2.1);
	stepperY.setRate(4000, true);
	stepperX.setRate(4000, true);
	stepperX.runForSteps(5000);
	stepperY.runForSteps(5000);
	Stepper::waitForAllSteppers();
	while(true){

		stepperY.setRate(Stepper::getSpeedForShorterAxle(3000, 10000, stepperX.getCurrentRate()));
		stepperX.runForSteps(10000);
		stepperY.runForSteps(3000);
		Stepper::waitForAllSteppers();
		vTaskDelay(1000);
		stepperY.toggleDirection();
		stepperX.toggleDirection();
	}
}

int main(void)
{
	prvSetupHardware();
	RunningTime::setup();


	/*PWMController* pwm = new PWMController(LPC_SCT0);
	pwm->initCounterL(10000, 50, true, 1);
	pwm->setOutputL(1, 1, 0, true);
	pwm->startCounterL();*/

	//PWMController servo = new PWMController(LPC_SCT0);
	//pwm->initCounterL();

	NVIC_EnableIRQ(MRT_IRQn);

	//xTaskCreate(PWMTest, "PWM", configMINIMAL_STACK_SIZE*2, pwm, (tskIDLE_PRIORITY + 1UL), nullptr);
	xTaskCreate(StepperTest, "stepperTest", configMINIMAL_STACK_SIZE*6, nullptr, (tskIDLE_PRIORITY + 2UL), nullptr);

	vTaskStartScheduler();

	return 1;
}

