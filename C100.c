/**
  ******************************************************************************
  * @file    C100.c
  * @author  Stian Sandve, UiS Subsea
  * @version V1.0.0
  * @date    04-Oct-2014
  * @brief   This file provides some of the C100 firmware functions.
 *******************************************************************************
 * Copyright (c) 2014, UiS Subsea
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of UiS Subsea nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */

 /* Includes ------------------------------------------------------------------*/
#include "C100.h"
#include <string.h>
#include <stdbool.h>

/**
  * @brief  Fills each C100_InitStruct member with its default value.
  * @param  C100_InitStruct: pointer to a C100_InitTypeDef structure
  *         which will be initialized.
  * @retval None
  */
void C100_StructInit(C100_InitTypeDef* C100_InitStruct)
{
	/* C100_InitStruct members default value */
  C100_InitStruct->C100_BaudRate = C100_BaudRate_48;
  C100_InitStruct->C100_MessageType = C100_MessageType_NMEA;
  C100_InitStruct->C100_MessageRate = C100_MessageRate_60;
}


/**
  * @brief  Send the commands needed to configure the KVH C100 compass
  *			according to the specified parameters in the C100_InitStruct.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   		This parameter can be one of the following values:
  *   		USART1, USART2, USART3, UART4 or UART5.
  * @param  C100_InitStruct: pointer to a C100_InitTypeDef structure
  *         that contains the configuration information for the specified C100
  *			compass.
  * @retval None
  */
void C100_Init(USART_TypeDef* USARTx, C100_InitTypeDef* C100_InitStruct)
{
	  assert_param(IS_USART_ALL_PERIPH(USARTx));
	  assert_param(IS_C100_BAUD_RATE(C100_InitStruct->C100_BaudRate));  
    assert_param(IS_C100_MESSAGE_TYPE(C100_InitStruct->C100_MessageType));
    assert_param(IS_C100_MESSAGE_RATE(C100_InitStruct->C100_MessageRate));

  	C100_USART_Init(USARTx, C100_InitStruct->C100_BaudRate);

  	USART_SendCommand(USARTx, C100_InitStruct->C100_BaudRate);
  	USART_SendCommand(USARTx, C100_InitStruct->C100_MessageType);
  	USART_SendCommand(USARTx, C100_InitStruct->C100_MessageRate);
}

/**
  * @brief  Initializes the USARTx peripheral according to the specified
  *         parameters in the USART_InitStruct.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   		This parameter can be one of the following values:
  *   		USART1, USART2, USART3, UART4 or UART5.
  * @param  C100_InitStruct: pointer to a C100_InitTypeDef structure
  *         that contains the configuration information for the specified C100
  *			compass.
  * @retval None
  */
void C100_USART_Init(USART_TypeDef* USARTx, uint32_t baudrate)
{
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	// Initialize USART structure
	USART_InitTypeDef USART_InitStructure;
	USART_StructInit(&USART_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;

	USART_Init(USARTx, &USART_InitStructure);
	USART_Cmd(USARTx, ENABLE);
}

/**
  * @brief  Reads one ASCII message from the C100 compass
  *			and extracts the heading. The heading is then converted
  *			from ASCII to an uint16_t. This function only retrives the
  *			most significant part of the heading and discards the part
  *			behind the comma.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   		This parameter can be one of the following values:
  *   		USART1, USART2, USART3, UART4 or UART5.
  * @retval An integer representation of the current heading reported by
  *			the compass.
  */
uint16_t C100_ReadHeading(USART_TypeDef* USARTx, MessageType type)
{
	assert_param(IS_USART_ALL_PERIPH(USARTx));

	int idx = 0;
	int hdg_idx = 0;
	char hdgBuffer* = C100_ReadMessage(USARTx);
	char atoi_buffer[3];
  char hdg_start;

  // The function does not support the XY message type yet
  switch(type)
  {
    case(MessageType.NMEA):
      hdg_start = C100_NMEA_HEADING_START;
      break;
    case(MessageType.KVH):
      hdg_start = C100_KVH_HEADING_START;
      break;
    default:
      hdg_start = C100_NMEA_HEADING_START;
      break;
  }
	
	// Locate the index of heading start
	for(idx = 0; idx < strlen(hdg_buffer); idx++)
	{
		if(hdg_buffer[idx] == C100_KVH_HEADING_START)
		{
			hdg_idx = idx + 1;
			break;
		}
	}

	// Put the ASCII heading in a buffer
	// We only use the most significant digits here,
	// the decimal is discarded.
	for(idx = 0; idx < 3; idx++)
	{
		atoi_buffer[idx] = rec_buffer[idx + hdg_idx];
	}

	// Convert ASCII heading to an integer
	hdg = atoi(atoi_buffer);

	// Extract the last 16 bits from the heading
	hdg = hdg & 0xffff;

	return hdg;
}

/**
  * @brief  Reads one ASCII message from the C100 compass.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   		This parameter can be one of the following values:
  *   		USART1, USART2, USART3, UART4 or UART5.
  * @retval An ASCII representation of the current heading.
  *			The form of the message depends on the message type
  *			specified during compass initialization.
  */
char* C100_ReadMessage(USART_TypeDef* USARTx)
{
	assert_param(IS_USART_ALL_PERIPH(USARTx));

	static uint8_t ascii_data;
	static bool done = false;
	static uint8_t idx = 0;
	char rec_buffer[20];

	// Read ASCII message from C100 compass
	while(!done)
	{
		ascii_data = USART_ReadChar(USARTx);
		rec_buffer[idx++] = ascii_data;

		if(ascii_data == CARRIAGE_RETURN)
			done = true;
	}

	return rec_buffer;
}

/**
  * @brief  Configure the C100 compass to use the specified message
  *         rate.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *       This parameter can be one of the following values:
  *       USART1, USART2, USART3, UART4 or UART5.
  * @param  C100_MessageRate: Specifies the message rate.
  *         This parameter can be any of the following values:
  *         C100_MessageRate_6, C100_MessageRate_60, C100_MessageRate_600.
  * @retval None
  */
bool C100_SetMessageRate(USART_TypeDef* USARTx, char* C100_MessageRate)
{
  assert_param(IS_C100_MESSAGE_RATE(C100_MessageRate));

  C100_SendCommand(USARTx, C100_MessageRate);

  uint8_t response = C100_ReadChar(USARTx);
  return response == C100_SUCCESS_RESPONSE);
}

/**
  * @brief  Configure the C100 compass to use the specified message
  *         type.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *       This parameter can be one of the following values:
  *       USART1, USART2, USART3, UART4 or UART5.
  * @param  C100_MessageRate: Specifies the message rate.
  *         This parameter can be any of the following values:
  *         C100_MessageType_NMEA, C100_MessageRate_KVH, 
  *         C100_MessageRate_XY_CORRECTED, C100_MessageRate_XY_UNCORRECTED.
  * @retval None
  */
bool C100_SetMessageType(USART_TypeDef* USARTx, char* C100_Message_Type)
{
  assert_param(IS_C100_MESSAGE_TYPE(C100_Message_Type));

  C100_SendCommand(USARTx, C100_Message_Type);

  uint8_t response = C100_ReadChar(USARTx);
  return response == C100_SUCCESS_RESPONSE);
}

/**
  * @brief  Configure the C100 compass to use the specified baud
  *         rate.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *       This parameter can be one of the following values:
  *       USART1, USART2, USART3, UART4 or UART5.
  * @param  C100_MessageRate: Specifies the message rate.
  *         This parameter can be any of the following values:
  *         C100_BaudRate_48, C100_BaudRate_96.
  * @retval None
  */
bool C100_SetBaudRate(USART_TypeDef* USARTx, char* C100_BaudRate)
{
  assert_param(IS_C100_BAUD_RATE(C100_BaudRate));

  C100_SendCommand(USARTx, C100_BaudRate);

  uint8_t response = C100_ReadChar(USARTx);
  return response == C100_SUCCESS_RESPONSE);
}

/**
  * @brief  Send an ASCII command to the C100 compass.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   		This parameter can be one of the following values:
  *   		USART1, USART2, USART3, UART4 or UART5.
  * @param  C100_Command: The user command that will be sent to
  *			the compass.
  * @retval None
  */
void C100_SendCommand(USART_TypeDef* USARTx, char* C100_Command)
{
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_C100_COMMAND(C100_Command));

	while(*C100_Command)
		USART_SendChar(USARTx, *C100_Command++);
}

/**
  * @brief  Send one character over USART to the C100 compass.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   		This parameter can be one of the following values:
  *   		USART1, USART2, USART3, UART4 or UART5.
  * @param  c: The character that will be sent to the compass.
  *			the compass.
  * @retval None
  */
void USART_SendChar(USART_TypeDef* USARTx, uint8_t c)
{
	assert_param(IS_USART_ALL_PERIPH(USARTx));

	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
	USART_SendData(USARTx, c);
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
}

/**
  * @brief  Read one character from the USART data register.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   		This parameter can be one of the following values:
  *   		USART1, USART2, USART3, UART4 or UART5.
  * @retval The character read from the specified USART.
  */
uint8_t USART_ReadChar(USART_TypeDef* USARTx)
{
	assert_param(IS_USART_ALL_PERIPH(USARTx));

	while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET);
	return USART_ReceiveData(USARTx) & 0xff;
}

