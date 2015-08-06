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

#include "stm32f4_usart.h"
#include "Memory.h"

void USARTx_Init(USART_Driver* USARTx)
{
  USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
  GPIO_InitTypeDef GPIO_InitStructure;
#ifdef USARTx_USE_DMA
	DMA_InitTypeDef DMA_InitStructure;
#endif

	// USARTx GPIO configuration
  // Enable GPIO clock
  USARTx->TX_GPIOClk(USARTx->TX_GPIOFunc, ENABLE);
	USARTx->RX_GPIOClk(USARTx->RX_GPIOFunc, ENABLE);
  // Connect USART pins to AF
	GPIO_PinAFConfig(USARTx->TX_GPIO, USARTx->TX_Src, USARTx->GPIO_AF_USART);
	GPIO_PinAFConfig(USARTx->RX_GPIO, USARTx->RX_Src, USARTx->GPIO_AF_USART);
  
  // Configure USART Tx and Rx as alternate function push-pull
	GPIO_StructInit(&GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Pin = USARTx->TX_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(USARTx->TX_GPIO, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = USARTx->RX_Pin;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(USARTx->RX_GPIO, &GPIO_InitStructure);
 
  // USARTx configuration
  // Enable the USART OverSampling by 8
  //USART_OverSampling8Cmd(USARTx->USART, ENABLE);
	// Enable USART clock
	USARTx->USART_CLK(USARTx->USART_Func, ENABLE);
	USART_StructInit(&USART_InitStructure);
	
	USART_InitStructure.USART_BaudRate = USARTx->USART_BaudRate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  // When using Parity the word length must be configured to 9 bits
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_ClockStructInit(&USART_ClockInitStruct);
	USART_ClockInit(USARTx->USART, &USART_ClockInitStruct);
  USART_Init(USARTx->USART, &USART_InitStructure);
		
	USART_ITConfig(USARTx->USART, USART_IT_TC, DISABLE);
	USART_ITConfig(USARTx->USART, USART_IT_IDLE, ENABLE);
	USART_ITConfig(USARTx->USART, USART_IT_RXNE, DISABLE);
	USART_ITConfig(USARTx->USART, USART_IT_TXE, DISABLE);
	
	NVIC_Init(&USARTx->NVIC_USART);

#ifdef USARTx_USE_DMA
  // Enable the DMA clock
  USARTx->DMA_CLK(USARTx->DMA_Func, ENABLE);
	NVIC_Init(&USARTx->NVIC_DMA_TX);
	//NVIC_Init(&USARTx->NVIC_DMA_RX);
	
  // Configure DMA controller to manage USART TX and RX DMA request
  // Configure DMA Initialization Structure
	// Configure TX DMA
	DMA_DeInit(USARTx->DMA_TX_Stream);
	DMA_InitStructure.DMA_Channel = USARTx->DMA_TX_CH;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USARTx->USART->DR));
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USARTx->DMA_TX_Buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = USARTx->DMA_TX_Size;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(USARTx->DMA_TX_Stream, &DMA_InitStructure);
	DMA_ITConfig(USARTx->DMA_TX_Stream, DMA_IT_TC, ENABLE);
	
  // Configure RX DMA
	DMA_DeInit(USARTx->DMA_RX_Stream);
	DMA_InitStructure.DMA_BufferSize = USARTx->DMA_RX_Size;
  DMA_InitStructure.DMA_Channel = USARTx->DMA_RX_CH;;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)USARTx->DMA_RX_Buffer; 
  DMA_Init(USARTx->DMA_RX_Stream, &DMA_InitStructure);
	DMA_Cmd(USARTx->DMA_RX_Stream, ENABLE);
	USART_DMACmd(USARTx->USART, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
#endif

  // Enable USART
  USART_Cmd(USARTx->USART, ENABLE);
}

void USARTx_DeInit(USART_Driver* USARTx)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = USARTx->TX_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(USARTx->TX_GPIO, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = USARTx->RX_Pin;
  GPIO_Init(USARTx->RX_GPIO, &GPIO_InitStructure);
	
#ifdef USARTx_USE_DMA
	// Deinitialize DMA Streams
	DMA_DeInit(USARTx->DMA_TX_Stream);
	while (DMA_GetCmdStatus(USARTx->DMA_TX_Stream) != DISABLE);
	DMA_Cmd(USARTx->DMA_TX_Stream, DISABLE);
	DMA_DeInit(USARTx->DMA_RX_Stream);
	while (DMA_GetCmdStatus(USARTx->DMA_RX_Stream) != DISABLE);
	DMA_Cmd(USARTx->DMA_RX_Stream, DISABLE);
#endif

}

void USARTx_SendByte(USART_Driver* USARTx, uint8_t byte)
{
	USART_SendData(USARTx->USART, byte);
  // Loop until the end of transmission
  while (USART_GetFlagStatus(USARTx->USART, USART_FLAG_TXE) == RESET);
}

void USARTx_SendBytes(USART_Driver* USARTx, uint8_t* buffer, uint8_t length)
{
	uint8_t i = 0;
	
	while(i++ < length){
		USART_SendData(USARTx->USART, buffer[i]);
		while (USART_GetFlagStatus(USARTx->USART, USART_FLAG_TXE) == RESET);
	}
}

#ifdef USARTx_USE_DMA
void USARTx_DMA_SendBytes(USART_Driver* USARTx, uint8_t* buffer, uint8_t length)
{
	FastMemCpy(USARTx->DMA_TX_Buffer, buffer, length);
	// Enable USARTx DMA TX Channel
	DMA_SetCurrDataCounter(USARTx->DMA_TX_Stream, length); 
	DMA_Cmd(USARTx->DMA_TX_Stream, ENABLE);
	//USARTx->DMA_TX_Stream->NDTR = length;
	//USARTx->DMA_TX_Stream->CR |= DMA_SxCR_EN;
}

#endif
