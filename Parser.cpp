/*
 * Parser.cpp
 *
 *  Created on: 20.10.2017
 *      Author: Ville
 */

#include "Parser.h"
#include "ITM_write.h"

Command Parser::getCommand() {
	uint32_t len = USB_receive(recvBuffer, RCV_BUFSIZE);
	recvBuffer[len] = '\0';
	ITM_write(reinterpret_cast<char*>(recvBuffer));
	char character = *recvBuffer;
	uint32_t integer;
	double x;
	double y;
	int nParams;
	if(character == 'M')
		nParams = sscanf(reinterpret_cast<char*>(recvBuffer+1), "%lu %lf", &integer, &x);
	else
		nParams = sscanf(reinterpret_cast<char*>(recvBuffer+1), "%lu X%lf Y%lf", &integer, &x, &y);

	if(nParams < 1){
		return Command {CODES::E,0,0};
	}

	if(character == 'M'){
		if(nParams == 2){
			if(integer == 1) return Command {CODES::M1, x};
			else if(integer == 4) return Command {CODES::M4, x};
		} else if(integer == 10) return Command {CODES::M10};

	} else if (character == 'G'){
		if(integer == 1)
			if(nParams == 4)
				return Command {CODES::G1, x, y};
		if(integer == 28)
			return Command{CODES::G28};

	}
	return Command {CODES::E,0,0};
}
