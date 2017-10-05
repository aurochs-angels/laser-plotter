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

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, false);
	Chip_SWM_Init();
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void Test1(void* x){
	SemaphoreHandle_t go = x;
	while(1){
		if(xSemaphoreTake(go, portTICK_PERIOD_MS*10)){
			Board_LED_Set(0, 1);
		} else {
			Board_LED_Set(0, 0);
		}
	}
}

void Test2(void* x){
	SemaphoreHandle_t go = x;
	while(1){
		if(xSemaphoreTake(go, portTICK_PERIOD_MS*10)){
			Board_LED_Set(1, 1);
		} else {
			Board_LED_Set(1, 0);
		}
	}
}

/* the following is required if runtime statistics are to be collected */
extern "C" {

void vConfigureTimerForRunTimeStats( void ) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}

void PWMTest(void* pPWM){
	PWMController pwm = *static_cast<PWMController*>(pPWM);
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

int main(void)
{
	prvSetupHardware();

	LimitSwitch* limit1 = new LimitSwitch("limit1", configMINIMAL_STACK_SIZE*3, (tskIDLE_PRIORITY + 1UL), 0, 27);
	LimitSwitch* limit2 = new LimitSwitch("limit2", configMINIMAL_STACK_SIZE*3, (tskIDLE_PRIORITY + 1UL), 0, 28);

	PWMController* pwm = new PWMController(LPC_SCT0);
	pwm->initCounterL(10000, 50, true);
	pwm->setOutputL(1, 1, 0, true);
	pwm->startCounterL();

	xTaskCreate(Test1, "test1", configMINIMAL_STACK_SIZE*3, limit1->getSemaphoreHandle(), (tskIDLE_PRIORITY + 1UL), nullptr);
	xTaskCreate(Test2, "test2", configMINIMAL_STACK_SIZE*3, limit2->getSemaphoreHandle(), (tskIDLE_PRIORITY + 1UL), nullptr);
	xTaskCreate(PWMTest, "PWM", configMINIMAL_STACK_SIZE*3, pwm, (tskIDLE_PRIORITY + 1UL), nullptr);


	vTaskStartScheduler();

	return 1;
}

