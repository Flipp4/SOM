/*
 * Communication.h
 *
 *  Created on: 09.06.2019
 *      Author: Filip
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "Common_data.h"
#include <stdint.h>

void Communication_initialize ( void );
enInternalProtocolCommand_t Communication_Process (uint8_t * Buffer);

#endif /* COMMUNICATION_H_ */
