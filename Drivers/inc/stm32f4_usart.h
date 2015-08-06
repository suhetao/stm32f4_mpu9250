/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __STM32F4_USART_H
#define __STM32F4_USART_H

// include
#include "stm32f4xx.h"
#include "stm32f4_rcc.h"

#define DEFAULT_BAUDRATE 115200
#define USARTx_USE_DMA

#ifdef USARTx_USE_DMA
#define DEFAULT_BUFFERSIZE 256
#endif

typedef struct USART_DRIVER_T
{
	USART_TypeDef* USART;
	RCC_AXXPeriphClockCmd USART_CLK;
	uint32_t USART_Func;
	uint32_t USART_BaudRate;
	
	GPIO_TypeDef* TX_GPIO;
	RCC_AXXPeriphClockCmd TX_GPIOClk;
	uint32_t TX_GPIOFunc;
	uint16_t TX_Pin;
	uint16_t TX_Src;
	
	GPIO_TypeDef* RX_GPIO;
	RCC_AXXPeriphClockCmd RX_GPIOClk;
	uint32_t RX_GPIOFunc;
	uint16_t RX_Pin;
	uint16_t RX_Src;
	
#ifdef USARTx_USE_DMA
	NVIC_InitTypeDef NVIC_USART;
	NVIC_InitTypeDef NVIC_DMA_TX;
	
	RCC_AXXPeriphClockCmd DMA_CLK;
	uint32_t DMA_Func;
	
	uint32_t DMA_TX_Size;
	uint8_t* DMA_TX_Buffer;
	DMA_Stream_TypeDef* DMA_TX_Stream;
	uint32_t DMA_TX_CH;
	
	uint32_t DMA_RX_Size;
	uint8_t* DMA_RX_Buffer;
	DMA_Stream_TypeDef* DMA_RX_Stream;
	uint32_t DMA_RX_CH;
#endif
	uint8_t GPIO_AF_USART;
	
}USART_Driver;

void USARTx_Init(USART_Driver* USARTx);
void USARTx_DeInit(USART_Driver* USARTx);

void USARTx_SendByte(USART_Driver* USARTx, uint8_t byte);
void USARTx_SendBytes(USART_Driver* USARTx, uint8_t* buffer, uint8_t length);
#ifdef USARTx_USE_DMA
void USARTx_DMA_SendBytes(USART_Driver* USARTx, uint8_t* buffer, uint8_t length);
#endif

#endif
