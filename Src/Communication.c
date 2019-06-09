/*
 * Communication.c
 *
 *  Created on: 09.06.2019
 *      Author: Filip
 */

#include "Communication.h"
#include <stdint.h>


void Communication_Initialize()
{
/*	Enable receive interrupt*/



}

enInternalProtocolCommand_t Communication_Process (uint8_t * Buffer)
{

	enInternalProtocolCommand_t ProcessResult = enInternProtocolCommand_Error;

	if(strcmp(Buffer, "AD1") == 0)
	{
		ProcessResult = enInternProtocolCommand_MeasureA;
	}
	else if(strcmp(Buffer, "OP1") == 0)
	{
		ProcessResult = enInternProtocolCommand_DataOperationA;
	}
	else if(strcmp(Buffer, "OP2") == 0)
	{
		ProcessResult = enInternProtocolCommand_DataOperationB;
	}
	else if(strcmp(Buffer, "AD2") == 0)
	{
		ProcessResult = enInternProtocolCommand_MeasureB;
	}
	else if(strcmp(Buffer, "AD3") == 0)
	{
		ProcessResult = enInternProtocolCommand_MeasureC;
	}
	else if(strcmp(Buffer, "RET") == 0)
	{
		ProcessResult = enInternProtocolCommand_Send;
	}
	else if(strcmp(Buffer, "CLR") == 0)
	{
		ProcessResult = enInternProtocolCommand_Clear;
	}
	else
	{
		ProcessResult = enInternProtocolCommand_Error;
	}


	return ProcessResult;
}
