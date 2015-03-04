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

#include "stm32f4_serial.h"
#include "stm32f4_usart.h"
#include "stm32f4_common.h"
#include "stm32f4_string.h"

#ifdef USARTx_USE_DMA
static uint8_t DMA_TxBuffer[DEFAULT_BUFFERSIZE];
static uint8_t DMA_RxBuffer[DEFAULT_BUFFERSIZE];
static uint8_t USARTx_Rx_Buffer[DEFAULT_BUFFERSIZE];
#endif

static USART_Driver Serial = {
	USART1, RCC_APB2PeriphClockCmd, RCC_APB2Periph_USART1, DEFAULT_BAUDRATE,
	GPIOA, RCC_AHB1PeriphClockCmd , RCC_AHB1Periph_GPIOA,
	GPIO_Pin_9, GPIO_PinSource9, GPIO_Pin_10, GPIO_PinSource10,
#ifdef USARTx_USE_DMA
	{ USART1_IRQn, 1, 0, ENABLE },
	{	DMA2_Stream7_IRQn, 1, 1, ENABLE },
	{	DMA2_Stream5_IRQn, 1, 2, ENABLE },
	
	RCC_AHB1PeriphClockCmd, RCC_AHB1Periph_DMA2,
	DEFAULT_BUFFERSIZE, DMA_TxBuffer, DMA2_Stream7,
	DMA_Channel_4, DMA_FLAG_TCIF7, DMA_FLAG_TEIF7,
	DEFAULT_BUFFERSIZE, DMA_RxBuffer, DMA2_Stream5,
	DMA_Channel_4, DMA_FLAG_TCIF5, DMA_FLAG_TEIF5,
#endif
	GPIO_AF_USART1
};
static USART_Driver* pSerial = &Serial;

void Serial_Init(void)
{
	USARTx_Init(pSerial);
}

void Serial_SendByte(uint8_t byte)
{
	USARTx_SendByte(pSerial, byte);
}

void Serial_SendBytes(uint8_t* buffer, uint8_t length)
{
#ifdef USARTx_USE_DMA
	USARTx_DMA_SendBytes(pSerial, buffer, length);
#else
	USARTx_SendBytes(pSerial, buffer, length);
#endif
}

void Serial_Upload(short accel[3], short gyro[3], short compass[3], long quat[4], long temperature, long pressure)
{
	//must bytes alignment
	uint8_t out[PACKET_LENGTH];
	short* sDest = (short*)out;
	long* lDest;
	//uint8_t count = 0;
	//memset(out, 0, PACKET_LENGTH);
	//out[0] = 0x55;
	//out[1] = 0xAA;
	
	*sDest++ = 0xAA55;
	//accel
	/*
	out[4] = (uint8_t)accel[0];
	out[5] = (uint8_t)(accel[0] >> 8);
	out[6] = (uint8_t)accel[1];
	out[7] = (uint8_t)(accel[1] >> 8);
	out[8] = (uint8_t)accel[2];
	out[9] = (uint8_t)(accel[2] >> 8);
	*/
	*sDest++ = accel[0];
	*sDest++ = accel[1];
	*sDest++ = accel[2];
	//gyro
	/*
	out[10] = (uint8_t)gyro[0];
	out[11] = (uint8_t)(gyro[0] >> 8);
	out[12] = (uint8_t)gyro[1];
	out[13] = (uint8_t)(gyro[1] >> 8);
	out[14] = (uint8_t)gyro[2];
	out[15] = (uint8_t)(gyro[2] >> 8);
	*/
	*sDest++ = gyro[0];
	*sDest++ = gyro[1];
	*sDest++ = gyro[2];
	//compass
	/*
	out[16] = (uint8_t)compass[0];
	out[17] = (uint8_t)(compass[0] >> 8);
	out[18] = (uint8_t)compass[1];
	out[19] = (uint8_t)(compass[1] >> 8);
	out[20] = (uint8_t)compass[2];
	out[21] = (uint8_t)(compass[2] >> 8);
	*/
	*sDest++ = compass[0];
	*sDest++ = compass[1];
	*sDest++ = compass[2];
	//quat
	/*
	out[22] = (uint8_t)quat[0];
	out[23] = (uint8_t)(quat[0] >> 8);
	out[24] = (uint8_t)(quat[0] >> 16);
	out[25] = (uint8_t)(quat[0] >> 24);
	out[26] = (uint8_t)quat[1];
	out[27] = (uint8_t)(quat[1] >> 8);
	out[28] = (uint8_t)(quat[1] >> 16);
	out[29] = (uint8_t)(quat[1] >> 24);
	out[30] = (uint8_t)quat[2];
	out[31] = (uint8_t)(quat[2] >> 8);
	out[32] = (uint8_t)(quat[2] >> 16);
	out[33] = (uint8_t)(quat[2] >> 24);
	out[34] = (uint8_t)quat[3];
	out[35] = (uint8_t)(quat[3] >> 8);
	out[36] = (uint8_t)(quat[3] >> 16);
	out[37] = (uint8_t)(quat[3] >> 24);
	*/
	lDest = (long*)sDest;
	*lDest++ = quat[0];
	*lDest++ = quat[1];
	*lDest++ = quat[2];
	*lDest++ = quat[3];
	//tempperature
	/*
	out[38] = (uint8_t)temperature;
	out[39] = (uint8_t)(temperature >> 8);
	out[40] = (uint8_t)(temperature >> 16);
	out[41] = (uint8_t)(temperature >> 24);
	*/
	*lDest++ = temperature;
	//pressure
	/*
	out[42] = (uint8_t)pressure;
	out[43] = (uint8_t)(pressure >> 8);
	out[44] = (uint8_t)(pressure >> 16);
	out[45] = (uint8_t)(pressure >> 24);
	*/
	*lDest++ = pressure;
	
	//out[44] = '\r';
	//out[45] = '\n';
	
	sDest = (short*)lDest;
	*sDest = 0x0A0D;
	
	Serial_SendBytes((uint8_t*)out, PACKET_LENGTH);
}

#ifdef USARTx_USE_DMA
//according to the hardware
//such as using usart1, DMA2_Stream7 for tx, DMA2_Stream5 for rx
void USART1_IRQHandler(void)                                 
{
	uint32_t usage = 0;

	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET){
		usage = USART1->SR;
		usage = USART1->DR;
		
		DMA_Cmd(DMA2_Stream5, DISABLE);
		usage = DEFAULT_BUFFERSIZE - DMA_GetCurrDataCounter(DMA2_Stream5);
		FastMemCpy(USARTx_Rx_Buffer, DMA_RxBuffer, usage);
		DMA_SetCurrDataCounter(DMA2_Stream5, DEFAULT_BUFFERSIZE);
		DMA_Cmd(DMA2_Stream5,ENABLE);
	}
}

//TX
void DMA2_Stream7_IRQHandler(void)  
{
	if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7)){
		DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
		DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TEIF7);
		DMA_Cmd(DMA2_Stream7, DISABLE);
	}	
}

//RX
void DMA2_Stream5_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
	DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
	DMA_Cmd(DMA2_Stream5, DISABLE);
	DMA_SetCurrDataCounter(DMA2_Stream5, DEFAULT_BUFFERSIZE);
	DMA_Cmd(DMA2_Stream5, ENABLE);
}

#endif
