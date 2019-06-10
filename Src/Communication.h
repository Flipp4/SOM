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

void Communication_initialize ( void* huart );
enInternalProtocolCommand_t Communication_Process (uint8_t * Buffer);

void Communication_ReceiveCallback(uint8_t Byte);

#endif /* COMMUNICATION_H_ */
