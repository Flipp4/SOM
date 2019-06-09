/*
 * Common_data.h
 *
 *  Created on: 05.06.2019
 *      Author: Filip
 */

#ifndef COMMON_DATA_H_
#define COMMON_DATA_H_

/*	Internal protocol command types - define style	*/
#define MEASURE 0x0A
#define MEAN	0x0B
#define STORE	0x0C

#define COMMAND_SIZE 0x03

/*	Internal protocol command types - enum style	*/
typedef enum InternProtocolCommand
{
	enInternProtocolCommand_MeasureA,
	enInternProtocolCommand_MeasureB,
	enInternProtocolCommand_MeasureC,
	enInternProtocolCommand_DataOperationA,
	enInternProtocolCommand_DataOperationB,
	enInternProtocolCommand_Send,
	enInternProtocolCommand_Clear,
	enInternProtocolCommand_Error
}enInternalProtocolCommand_t;



#endif /* COMMON_DATA_H_ */
