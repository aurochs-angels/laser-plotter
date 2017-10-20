/*
 * Parser.h
 *
 *  Created on: 20.10.2017
 *      Author: Ville
 */

#ifndef PARSER_H_
#define PARSER_H_

#include "FreeRTOS.h"
#include "user_vcom.h"
#include <cstring>
#include <cstdint>
#include "queue.h"

enum class CODES {
	G28, M1, M4, M10, G1, E
};

struct Command {
	CODES code;
	double x;
	double y;
};

class Parser {
public:
	Command getCommand();
private:
	uint8_t recvBuffer[RCV_BUFSIZE];
};

#endif /* PARSER_H_ */
