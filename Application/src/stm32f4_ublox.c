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

#include "stm32f4_ublox.h"
#include "stm32f4_usart.h"
//
#include "Map.h"
#include "Memory.h"
#include "Fifo.h"

//ungly implement: the max size of the data
//max(sizeof(nmeaGPGGA), sizeof(nmeaGPGSA), sizeof(nmeaGPRMC), sizeof(nmeaGPVTG))
#define UBLOX_MAX_MESSAGESIZE sizeof(nmeaGPGGA)

//////////////////////////////////////////////////////////////////////////
//UBlox Driver
#ifdef USARTx_USE_DMA
static uint8_t DMA_TxBuffer[UBLOX_DEFAULT_TX_BUFFERSIZE];
static uint8_t DMA_RxBuffer[UBLOX_DEFAULT_RX_BUFFERSIZE];
static uint8_t USARTx_Rx_Buffer[UBLOX_DEFAULT_RX_BUFFERSIZE];
#endif

static USART_Driver Ublox = {
	UART4, RCC_APB1PeriphClockCmd, RCC_APB1Periph_UART4, UBLOX_DEFAULT_BAUDRATE,
	GPIOC, RCC_AHB1PeriphClockCmd , RCC_AHB1Periph_GPIOC, GPIO_Pin_10, GPIO_PinSource10,
	GPIOC, RCC_AHB1PeriphClockCmd , RCC_AHB1Periph_GPIOC, GPIO_Pin_11, GPIO_PinSource11,
#ifdef USARTx_USE_DMA
	{ UART4_IRQn, 1, 2, ENABLE },
	{	DMA1_Stream4_IRQn, 1, 3, ENABLE },

	RCC_AHB1PeriphClockCmd, RCC_AHB1Periph_DMA2,
	UBLOX_DEFAULT_TX_BUFFERSIZE, DMA_TxBuffer, DMA1_Stream4, DMA_Channel_4,
	UBLOX_DEFAULT_RX_BUFFERSIZE, DMA_RxBuffer, DMA1_Stream2, DMA_Channel_4,
#endif
	GPIO_AF_UART4
};
//
static USART_Driver* pUblox = &Ublox;
//////////////////////////////////////////////////////////////////////////
//Fifo
static Fifo UbloxFifo;
static Fifo* pUbloxFifo = &UbloxFifo;
//////////////////////////////////////////////////////////////////////////
//Ublox GPS Parser
static s8 _ParserBuff[UBLOX_DEFAULT_PARSER_MAXSIZE];
static Ublox_ParserBuff ParserBuffer = {_ParserBuff, UBLOX_DEFAULT_PARSER_MAXSIZE, UBLOX_DEFAULT_PARSER_MAXSIZE, 0}; 
static s8 Message[UBLOX_MAX_MESSAGESIZE];
static nmeaINFO uBloxInfo = {0};

static Map UbloxMap = {0};
static Map *pUbloxMap = &UbloxMap;
//////////////////////////////////////////////////////////////////////////
void Ublox_Init(void)
{
	Fifo_Init(pUbloxFifo, USARTx_Rx_Buffer, UBLOX_DEFAULT_RX_BUFFERSIZE);
	USARTx_Init(pUblox);
	//must init with local two reference points including lat, lon and the length
	//betweeb two reference points 
	//Map_Init(pUbloxMap, first_lat, first_lon, second_lat, second_lon, length);
}

void Ublox_SendBytes(uint8_t* buffer, uint8_t length)
{
#ifdef USARTx_USE_DMA
	USARTx_DMA_SendBytes(pUblox, buffer, length);
#else
	USARTx_SendBytes(pUblox, buffer, length);
#endif
}

void Ublox_ParserMessage()
{
	u16 useSize;
	//////////////////////////////////////////////////////////////////////////
	//read fully UBLOX_DEFAULT_PARSER_MAXSIZE from fifo
	useSize = Fifo_Get(pUbloxFifo, (u8*)(ParserBuffer.Data + ParserBuffer.Left), ParserBuffer.Need);
	ParserBuffer.Need -= useSize;
	ParserBuffer.Left += useSize;
	if(0 == ParserBuffer.Need){
		useSize = NEMA_Parser(ParserBuffer.Data, UBLOX_DEFAULT_PARSER_MAXSIZE);
		ParserBuffer.Left = UBLOX_DEFAULT_PARSER_MAXSIZE - useSize;
		MemMove((u8*)ParserBuffer.Data, (u8*)(ParserBuffer.Data + useSize), ParserBuffer.Left);
		ParserBuffer.Need = useSize;
	}
}

void Ublox_GetMessage()
{
	s32 iMessageType;
	s16 ret = 0;
	Ublox_ParserMessage();
	ret = NEMA_GetMessage(Message, &iMessageType);
	if(ret <= 0){
		return;
	}
	switch(iMessageType){
		case GPNON:
			break;
		case GPGGA:
			uBloxInfo.lat = NMEA_Convert2Degrees(((PnmeaGPGGA)Message)->lat);
			uBloxInfo.lon = NMEA_Convert2Degrees(((PnmeaGPGGA)Message)->lon);
			uBloxInfo.alt = ((PnmeaGPGGA)Message)->alt;
			break;
		case GPGSA:
			break;
		case GPRMC:
			uBloxInfo.lat = NMEA_Convert2Degrees(((PnmeaGPRMC)Message)->lat);
			uBloxInfo.lon = NMEA_Convert2Degrees(((PnmeaGPRMC)Message)->lon);
			uBloxInfo.spd = ((PnmeaGPRMC)Message)->spd;
			uBloxInfo.cog = ((PnmeaGPRMC)Message)->cog;
			break;
		case GPVTG:
			break;
	}
}

void Ublox_GetPostion(double *x, double *y, double *z)
{
	Map_GetXY(pUbloxMap, uBloxInfo.lat, uBloxInfo.lon, x, y);
	*z = uBloxInfo.alt;
}

#ifdef USARTx_USE_DMA
//according to the hardware
//such as using uart4, DMA1_Stream4 for tx, DMA1_Stream2 for rx
void UART4_IRQHandler(void)                                 
{
	u16 DATA_LEN = 0;
	if(USART_GetITStatus(UART4, USART_IT_TC) != RESET){
		USART_ITConfig(UART4, USART_IT_TC, DISABLE);
	}
	else if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET){
		UART4->SR;
		UART4->DR;

		DMA_Cmd(DMA1_Stream2, DISABLE);
		DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
		DATA_LEN = UBLOX_DEFAULT_RX_BUFFERSIZE - DMA_GetCurrDataCounter(DMA1_Stream2);
		//////////////////////////////////////////////////////////////////////////
		//put DMA data to FIFO
		Fifo_Put(pUbloxFifo, DMA_RxBuffer, DATA_LEN);
		//////////////////////////////////////////////////////////////////////////
		DMA_SetCurrDataCounter(DMA1_Stream2, DEFAULT_BUFFERSIZE);
		DMA_Cmd(DMA1_Stream2, ENABLE);
	}
}

//TX
void DMA1_Stream4_IRQHandler(void)  
{
	if(DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4)){
		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
		DMA_Cmd(DMA1_Stream4, DISABLE);
		USART_ITConfig(UART4, USART_IT_TC, ENABLE);
	}	
}

#endif

