/*
 * Command.h
 *
 *  Created on: Oct 18, 2017
 *      Author: aino
 */

#ifndef COMMAND_H_
#define COMMAND_H_

class Command{
	enum type{home, calibrate, move, pencil, laser, error};
	int xCoord;
	int yCoord;
	type command;
};



#endif /* COMMAND_H_ */
