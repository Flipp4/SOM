/*
 * Communication.c
 *
 *  Created on: 09.06.2019
 *      Author: Filip
 */

#include "Communication.h"
#include <stdint.h>
#include "stm32f3xx.h"

#ifndef __STM32F3xx_HAL_UART_H
#include "stm32f3xx_hal_uart.h"
#endif

void Communication_Approve		( void );
void Communication_Clear 		( void );

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
	UART_HandleTypeDef * UartHandle;

}CommunicationRoot_t;


static CommunicationRoot_t Root =
{
		.CommunicationStage = enCommunication_Stage_FirstByte,
		.ReceiveBuffer = {0, 0, 0},
		.ByteCounter = 0,
		.ReceivedByte = 0
};

void Communication_Initialize( UART_HandleTypeDef * huart)
{
/*	Enable receive interrupt service*/

	HAL_UART_Receive_IT(huart, &Root.ReceivedByte, 1);
	Root.UartHandle = huart;

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Communication_ReceiveCallback(Root.ReceivedByte);
}

void Communication_ReceiveCallback(uint8_t Byte)
{

	switch(Root.CommunicationStage)
	{
	case enCommunication_Stage_FirstByte:
		HAL_UART_Receive_IT(&Root.UartHandle, &Root.ReceivedByte, 1);
		break;
	case enCommunication_Stage_SecondByte:
		HAL_UART_Receive_IT(&Root.UartHandle, &Root.ReceivedByte, 1);
		break;
	case enCommunication_Stage_ThirdByte:
		Communication_Approve();
		break;
	case enCommunication_Stage_Error:
		Communication_Clear();
		break;
	default:
		break;
	}

	return;

}
