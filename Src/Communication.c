/*
 * Communication.c
 *
 *  Created on: 09.06.2019
 *      Author: Filip
 */

#include "Communication.h"
#include <stdint.h>
#include "stm32f3xx.h"

typedef enum enCommunicationStage
{
	enCommunication_Stage_FirstByte,
	enCommunication_Stage_SecondByte,
	enCommunication_Stage_ThirdByte,
	enCommunication_Stage_Error

}enCommunicationStage_t;

typedef struct CommunicationRoot
{
	enCommunicationStage_t CommunicationStage;
	uint8_t ReceiveBuffer[3];
	uint8_t ByteCounter;
	uint8_t ReceivedByte;

}CommunicationRoot_t;


static CommunicationRoot_t Root =
{
		.CommunicationStage = enCommunication_Stage_FirstByte,
		.ReceiveBuffer = {0, 0, 0},
		.ByteCounter = 0,
		.ReceivedByte = 0
};

void Communication_Initialize()
{
/*	Enable receive interrupt service*/

	HAL_UART_Receive_IT(&huart1, Root->ReceivedByte, 1);



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

void Communication_ReceiveCallback(uint8_t Byte)
{

	switch(Root.CommunicationStage)
	{
	case enCommunication_Stage_FirstByte:
		break;
	case enCommunication_Stage_SecondByte:
		break;
	case enCommunication_Stage_ThirdByte:
		break;
	case enCommunication_Stage_Error:
		break;
	default:
		break;
	}


}
