/**
  ******************************************************************************
  * @file    C100.h
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

#ifndef _C100_H_
#define _C100_H_

/* Includes ------------------------------------------------------------------*/
#include <stm32f10x_usart.h>
/* ---------------------------------------------------------------------------*/

/* C100 user commands --------------------------------------------------------*/
#define C100_CMD_MESSAGE_RATE_6 				"=r,6\r"
#define C100_CMD_MESSAGE_RATE_60 				"=r,60\r"
#define C100_CMD_MESSAGE_RATE_600				"=r,600\r"
#define C100_CMD_MESSAGE_TYPE_NMEA				"=t,0\r"
#define C100_CMD_MESSAGE_TYPE_KVH				"=t,1\r"
#define C100_CMD_MESSAGE_TYPE_XY_CORRECTED		"=t,2\r"
#define C100_CMD_MESSAGE_TYPE_XY_UNCORRECTED	"=t,3\r"
#define C100_CMD_BAUD_RATE_96					"=b,96\r"
#define C100_CMD_BAUD_RATE_48					"=b,48\r"
/* ---------------------------------------------------------------------------*/

/* C100 response commands ----------------------------------------------------*/
#define C100_SUCCESS_RESPONSE	'>'
#define C100_HEADING_START		'$'
#define CARRIAGE_RETURN			0x0d
#define LINE_FEED				0x0a
/* ---------------------------------------------------------------------------*/

typedef struct
{
  uint32_t C100_BaudRate;            /*!< This member configures the USART communication baud rate.
                                           The baud rate is computed using the following formula:
                                            - IntegerDivider = ((PCLKx) / (16 * (USART_InitStruct->USART_BaudRate)))
                                            - FractionalDivider = ((IntegerDivider - ((u32) IntegerDivider)) * 16) + 0.5 */

  char* C100_MessageType;          /*!< Specifies the type of the messages received.
                                           This parameter can be a value of @ref C100_MessageType */

  char* C100_MessageRate;            /*!< Specifies the number of messages per minute.
                                           This parameter can be a value of @ref C100_MessageRate */
} C100_InitTypeDef;

/** @defgroup C100_BaudRate
  * @{
  */ 
  
#define C100_BaudRate_48			C100_CMD_BAUD_RATE_48
#define C100_BaudRate_96			C100_CMD_BAUD_RATE_96
#define IS_C100_BAUD_RATE(BAUD_RATE) (((BAUD_RATE) == C100_BaudRate_48) || \
                                     ((BAUD_RATE) == C100_BaudRate_96))
/**
  * @}
  */

/** @defgroup C100_MessageType
  * @{
  */ 
  
#define C100_MessageType_NMEA 			C100_CMD_MESSAGE_TYPE_NMEA
#define C100_MessageType_KVH			C100_CMD_MESSAGE_TYPE_KVH
#define C100_MessageType_XY_CORRECTED	C100_CMD_MESSAGE_TYPE_XY_CORRECTED
#define C100_MessageType_XY_UNCORRECTED	C100_CMD_MESSAGE_TYPE_XY_UNCORRECTED
#define IS_C100_MESSAGE_TYPE(MESSAGE_TYPE) (((MESSAGE_TYPE) == C100_MessageType_NMEA) || \
                                     ((MESSAGE_TYPE) == C100_MessageType_KVH) || \
                                     ((MESSAGE_TYPE) == C100_MessageType_XY_CORRECTED) || \
                                     ((MESSAGE_TYPE) == C100_MessageType_XY_UNCORRECTED))
/**
  * @}
  */

/** @defgroup C100_MessageRate
  * @{
  */ 
  
#define C100_MessageRate_6 			C100_CMD_MESSAGE_RATE_6
#define C100_MessageRate_60			C100_CMD_MESSAGE_RATE_60
#define C100_MessageRate_600		C100_CMD_MESSAGE_RATE_600
#define IS_C100_MESSAGE_RATE(MESSAGE_RATE) (((MESSAGE_RATE) == C100_MessageRate_6) || \
                                     ((MESSAGE_RATE) == C100_MessageRate_60) || \
                                     ((MESSAGE_RATE) == C100_MessageRate_600))
/**
  * @}
  */ 

/** @defgroup C100_UserCommand
  * @{
  */ 

#define IS_C100_USER_COMMAND(USER_COMMAND) (((USER_COMMAND) == C100_CMD_MESSAGE_RATE_6) || \
                                     ((USER_COMMAND) == C100_CMD_MESSAGE_RATE_60) || \
                                     (((USER_COMMAND) == C100_CMD_MESSAGE_RATE_600) || \
                                     ((USER_COMMAND) == C100_CMD_MESSAGE_TYPE_NMEA) || \
                                     (((USER_COMMAND) == C100_CMD_MESSAGE_TYPE_KVH) || \
                                     ((USER_COMMAND) == C100_CMD_MESSAGE_TYPE_XY_CORRECTED) || \
                                     (((USER_COMMAND) == C100_CMD_MESSAGE_TYPE_XY_UNCORRECTED) || \
                                     ((USER_COMMAND) == C100_CMD_BAUD_RATE_48) || \
                                     ((USER_COMMAND) == C100_CMD_BAUD_RATE_96))
/**
  * @}
  */ 

void C100_Init(USART_TypeDef* USARTx, C100_InitTypeDef* C100_InitStruct);
void C100_StructInit(C100_InitTypeDef* C100_InitStruct);
void C100_USART_Init(USART_TypeDef* USARTx);
char* C100_ReadMessage();
uint16_t C100_ReadHeading(USART_TypeDef* USARTx);
void C100_SendCommand(USART_TypeDef* USARTx, char* command);
void USART_SendChar(USART_TypeDef* USARTx, uint8_t c);
uint8_t USART_ReadChar(USART_TypeDef* USARTx);

#endif