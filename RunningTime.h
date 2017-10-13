/*
 * RunningTime.h
 *
 *  Created on: 10.10.2017
 *      Author: Ville
 */

#ifndef RUNNINGTIME_H_
#define RUNNINGTIME_H_
#include <cstdint>

class RunningTime {
public:
	static void setup();
	static void start();
	static void stop();
	static uint32_t getTime();
};

#endif /* RUNNINGTIME_H_ */
