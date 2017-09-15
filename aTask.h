/*
 * Task.h
 *
 *  Created on: 14.9.2017
 *      Author: Ville
 */

#ifndef ATASK_H_
#define ATASK_H_

#include "FreeRTOS.h"
#include "task.h"

class Task {
public:
	Task(const char* taskname, uint16_t stacksize, UBaseType_t priority){
		xTaskCreate(task, taskname, stacksize, this, (tskIDLE_PRIORITY + 1UL), &taskHandle);
	}
	~Task(){
		vTaskDelete(taskHandle);
	}
	inline static void task(void* thisPtr){
		static_cast<Task*>(thisPtr)->_task();
	}
	inline TaskHandle_t& getTaskHandle(){
		return taskHandle;
	}
private:
	TaskHandle_t taskHandle = NULL;
	virtual void _task() = 0;
};

#endif /* ATASK_H_ */
