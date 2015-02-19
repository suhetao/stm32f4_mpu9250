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

#include "stm32f4_spi.h"

static uint8_t DUMMY_BYTE = 0xA5;

void SPIx_Init(SPI_Driver* SPIx)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
#ifdef SPIx_USE_DMA
	DMA_InitTypeDef DMA_InitStructure;
#endif
	
	// Enable SPI and GPIO clocks
	SPIx->SPI_CLK(SPIx->SPI_Func, ENABLE);
	SPIx->GPIO_CLK(SPIx->GPIO_Func, ENABLE);

	// Connect SPI pins to AF
	GPIO_PinAFConfig(SPIx->Gpio, SPIx->SCK_Src, SPIx->GPIO_AF_SPI);
	GPIO_PinAFConfig(SPIx->Gpio, SPIx->MISO_Src, SPIx->GPIO_AF_SPI);
	GPIO_PinAFConfig(SPIx->Gpio, SPIx->MOSI_Src, SPIx->GPIO_AF_SPI);

	// SPI SCK/MISO/MOSI pin configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = SPIx->SCK_Pin | SPIx->MISO_Pin | SPIx->MOSI_Pin;
	GPIO_Init(SPIx->Gpio, &GPIO_InitStructure);

	// Configure GPIO PIN for Chip select
	GPIO_InitStructure.GPIO_Pin = SPIx->CS_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPIx->Gpio, &GPIO_InitStructure);

	// Chip DeSelect high
	CHIP_DESELECT(SPIx);

	//SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPIx->SPI);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPIx->SPI_Prescaler;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPIx->SPI, &SPI_InitStructure);

	SPI_CalculateCRC(SPIx->SPI, DISABLE);

	//Enable SPI
	SPI_Cmd(SPIx->SPI, ENABLE);
	while (SPI_I2S_GetFlagStatus(SPIx->SPI, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_ReceiveData(SPIx->SPI);
	
#ifdef SPIx_USE_DMA
	// Enable DMA clock
	SPIx->DMA_CLK(SPIx->DMA_Func, ENABLE);
	//Enable the SPIx_RX_DMA_CHANNEL (SPIx_RX) Interrupt
	NVIC_Init(&SPIx->NVIC_DMA_TX);

	//Enable the SPIx_TX_DMA_CHANNEL (SPIx_TX) Interrupt
	NVIC_Init(&SPIx->NVIC_DMA_RX);

	// Deinitialize DMA Streams
	DMA_DeInit(SPIx->DMA_TX_Stream);
	while (DMA_GetCmdStatus(SPIx->DMA_TX_Stream) != DISABLE);
	DMA_Cmd(SPIx->DMA_TX_Stream, DISABLE);
	DMA_DeInit(SPIx->DMA_RX_Stream);
	while (DMA_GetCmdStatus(SPIx->DMA_RX_Stream) != DISABLE);
	DMA_Cmd(SPIx->DMA_RX_Stream, DISABLE);

	// Configure DMA Initialization Structure
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPIx->SPI->DR));
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	// Configure TX DMA
	DMA_InitStructure.DMA_Channel = SPIx->DMA_TX_CH;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)0;
	DMA_Init(SPIx->DMA_TX_Stream, &DMA_InitStructure);
	// Configure RX DMA
	DMA_InitStructure.DMA_Channel = SPIx->DMA_RX_CH;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)0; 
	DMA_Init(SPIx->DMA_RX_Stream, &DMA_InitStructure);

	SPI_DMACmd(SPIx->SPI, SPI_DMAReq_Rx, ENABLE);
	SPI_DMACmd(SPIx->SPI, SPI_DMAReq_Tx, ENABLE);

	DMA_ClearFlag(SPIx->DMA_TX_Stream, SPIx->DMA_TX_Flag);
	DMA_ClearFlag(SPIx->DMA_RX_Stream, SPIx->DMA_RX_Flag);

	DMA_ITConfig(SPIx->DMA_TX_Stream, DMA_IT_TC | DMA_IT_TE, ENABLE);
	DMA_ITConfig(SPIx->DMA_RX_Stream, DMA_IT_TC | DMA_IT_TE, ENABLE);

	DMA_Cmd(SPIx->DMA_TX_Stream, DISABLE);
	DMA_Cmd(SPIx->DMA_RX_Stream, DISABLE);
#endif
}

void SPIx_DeInit(SPI_Driver* SPIx)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = SPIx->SCK_Pin | SPIx->MISO_Pin | SPIx->MOSI_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(SPIx->Gpio, &GPIO_InitStructure);  
  
  GPIO_InitStructure.GPIO_Pin = SPIx->CS_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init(SPIx->Gpio, &GPIO_InitStructure);
	
#ifdef SPIx_USE_DMA
	// Deinitialize DMA Streams
	DMA_DeInit(SPIx->DMA_TX_Stream);
	while (DMA_GetCmdStatus(SPIx->DMA_TX_Stream) != DISABLE);
	DMA_Cmd(SPIx->DMA_TX_Stream, DISABLE);
	DMA_DeInit(SPIx->DMA_RX_Stream);
	while (DMA_GetCmdStatus(SPIx->DMA_RX_Stream) != DISABLE);
	DMA_Cmd(SPIx->DMA_RX_Stream, DISABLE);
#endif
}

uint8_t SPIx_Read_Reg(SPI_Driver* SPIx, uint8_t reg)
{
	uint8_t tmp;

	reg += 0x80; //reading procedure has to set the most significant bit
	// Chip Select low 
	CHIP_SELECT(SPIx);

	// Send Register Address
	SPIx_SendByte(SPIx, reg);
	// Read a byte
	tmp = SPIx_SendByte(SPIx, DUMMY_BYTE);  

	// Chip Select high
	CHIP_DESELECT(SPIx);

	return tmp;
}

void SPIx_Write_Reg(SPI_Driver* SPIx, uint8_t regAddr, uint8_t data) {

	// Chip Select low 
	CHIP_SELECT(SPIx);

	// Send Register Address
	SPIx_SendByte(SPIx, regAddr);
	// Write a byte
	SPIx_SendByte(SPIx, data);  

	// Chip Select high
	CHIP_DESELECT(SPIx);
}

void SPIx_Read_Regs(SPI_Driver* SPIx, uint8_t regAddr, uint8_t length, uint8_t* buffer)
{
	uint8_t i = 0;

	regAddr += 0xc0; //reading procedure has to set the most significant bit
	// Chip Select low 
	CHIP_SELECT(SPIx);

	// Send Register Address
	SPIx_SendByte(SPIx, regAddr);

	while(i < length){
		// Read a byte
		buffer[i] = SPIx_SendByte(SPIx, DUMMY_BYTE);  
		i++;
	}
	// Chip Select high
	CHIP_DESELECT(SPIx);
}

#ifdef SPIx_USE_DMA
void SPIx_DMA_Read_Regs(SPI_Driver* SPIx, uint8_t regAddr, uint8_t length, uint8_t* buffer)
{
	regAddr += 0xc0; //reading procedure has to set the most significant bit
	// Chip Select low 
	CHIP_SELECT(SPIx);
	
	// Send Register Address
	SPIx_SendByte(SPIx, regAddr);

	SPIx->DMA_RX_Stream->CR &= 0xFFFFFFFE;
	while(SPIx->DMA_RX_Stream->CR & 0x00000001);
	SPIx->DMA_RX->LIFCR |= 0x0F400000;
	SPIx->DMA_RX_Stream->NDTR = length;
	SPIx->DMA_RX_Stream->M0AR = (uint32_t)buffer;
	SPIx->DMA_RX_Stream->CR |= 1;

	//Dummy TX channel configuration
	SPIx->DMA_TX_Stream->CR &= 0xFFFFFFFE;
	while(SPIx->DMA_TX_Stream->CR & 0x00000001);
	SPIx->DMA_TX->HIFCR |= 0x0000003D;
	SPIx->DMA_TX_Stream->CR &= 0xFFFFFBFF;//DMA_SxCR_MINC
	SPIx->DMA_TX_Stream->NDTR = length;
	SPIx->DMA_TX_Stream->M0AR = (u32)(&DUMMY_BYTE);
	SPIx->DMA_TX_Stream->CR |= 1;
}
#endif

uint8_t SPIx_SendByte(SPI_Driver* SPIx, uint8_t byte) {

	// Loop while DR register in not emplty
	while (SPI_I2S_GetFlagStatus(SPIx->SPI, SPI_I2S_FLAG_TXE) == RESET);
	// Send byte through the SPI1 peripheral
	SPI_I2S_SendData(SPIx->SPI, byte);
	// Wait to receive a byte
	while (SPI_I2S_GetFlagStatus(SPIx->SPI, SPI_I2S_FLAG_RXNE) == RESET);
	// Return the byte read from the SPI bus
	return SPI_I2S_ReceiveData(SPIx->SPI);
}

void SPIx_SetDivisor(SPI_Driver* SPIx, uint16_t Prescaler)
{
	uint16_t tmp;
	
	if(SPIx->SPI_Prescaler == Prescaler){
		return;
	}
	SPI_Cmd(SPIx->SPI, DISABLE);

	tmp = SPIx->SPI->CR1;
	tmp &= SPIx_BR_CLEAR_MASK;
	tmp |= Prescaler;

	SPIx->SPI->CR1 = tmp;
	SPI_Cmd(SPIx->SPI, ENABLE);
}
