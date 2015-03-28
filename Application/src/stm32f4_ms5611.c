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

#include "stm32f4_ms5611.h"
#include "stm32f4_spi.h"
#include "stm32f4_delay.h"

//////////////////////////////////////////////////////////////////////////
//basic SPI driver for ms5611
static SPI_Driver mMS5611 = {
	SPI1, RCC_APB2PeriphClockCmd, RCC_APB2Periph_SPI1,
	GPIOA, RCC_AHB1PeriphClockCmd, RCC_AHB1Periph_GPIOA,
	GPIOA, RCC_AHB1PeriphClockCmd, RCC_AHB1Periph_GPIOA, GPIO_Pin_4,
	GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7,
	GPIO_PinSource5, GPIO_PinSource6, GPIO_PinSource7,	
#ifdef SPIx_USE_DMA

#endif
	SPI_BaudRatePrescaler_4, GPIO_AF_SPI1
};
static SPI_Driver* pMS5611 = &mMS5611;

__IO u16 MS5611_C1 = 0, MS5611_C2 = 0, MS5611_C3 = 0, MS5611_C4 = 0, MS5611_C5 = 0, MS5611_C6 = 0;

void MS5611_Reset(SPI_Driver *MS5611)
{
	// Chip Select low 
	CHIP_SELECT(MS5611);
	SPIx_SendByte(MS5611, MS5611_RESET);
	Delay_Ms(3); //2.8ms reload from datasheet
	// Chip Select high
	CHIP_DESELECT(MS5611);
}

u16 MS5611_SPIx_ReadWord(SPI_Driver *MS5611, u8 addr)
{
	u8 data[2] = {0};
	u16 value = 0;

	// Chip Select low 
	CHIP_SELECT(MS5611);
	SPIx_SendByte(MS5611, addr);
	data[0] = SPIx_SendByte(MS5611, MS5611_READ);
	data[1] = SPIx_SendByte(MS5611, MS5611_READ);
	// Chip Select high
	CHIP_DESELECT(MS5611);

	value = data[0] << 8 | data[1];
	return value;
}

void MS5611_SPIx_ReadADC(SPI_Driver *MS5611, u8 osr, u32* value)
{
	u8 data[3] = {0};

	// Chip Select low 
	CHIP_SELECT(MS5611);
	SPIx_SendByte(MS5611, osr);
	// Chip Select high
	CHIP_DESELECT(MS5611);

	Delay_Ms(1);

	// Chip Select low 
	CHIP_SELECT(MS5611);
	SPIx_SendByte(MS5611, MS5611_READ_ADC);
	data[0] = SPIx_SendByte(MS5611, MS5611_READ_ADC);
	data[1] = SPIx_SendByte(MS5611, MS5611_READ_ADC);
	data[2] = SPIx_SendByte(MS5611, MS5611_READ_ADC);
	// Chip Select high
	CHIP_DESELECT(MS5611);

	*value = data[0] << 16 | data[1] << 8 | data[2];
}

void MS5611_ReadPROM(SPI_Driver *MS5611)
{
	// Read Calibration Data C1
	MS5611_C1 = MS5611_SPIx_ReadWord(MS5611, MS5611_READ_PROM_C1);
	Delay_Ms(1);
	// Read Calibration Data C2
	MS5611_C2 = MS5611_SPIx_ReadWord(MS5611, MS5611_READ_PROM_C2);
	Delay_Ms(1);
	// Read Calibration Data C3
	MS5611_C3 = MS5611_SPIx_ReadWord(MS5611, MS5611_READ_PROM_C3);
	Delay_Ms(1);
	// Read Calibration Data C4
	MS5611_C4 = MS5611_SPIx_ReadWord(MS5611, MS5611_READ_PROM_C4);
	Delay_Ms(1);
	// Read Calibration Data C5
	MS5611_C5 = MS5611_SPIx_ReadWord(MS5611, MS5611_READ_PROM_C5);
	Delay_Ms(1);
	// Read Calibration Data C6
	MS5611_C6 = MS5611_SPIx_ReadWord(MS5611, MS5611_READ_PROM_C6);
	Delay_Ms(1);
}

void MS5611_GetTemperatureAndPressure(s32* TEMP, s32 *P)
{
	s32 D1, D2;
	int64_t OFF, SENS, dT;

	MS5611_SPIx_ReadADC(pMS5611, D1_OSR_256, (u32*)&D1);
	MS5611_SPIx_ReadADC(pMS5611, D2_OSR_256, (u32*)&D2);

	dT = (int64_t)D2 - (int64_t)MS5611_C5 * (1<<8);
	*TEMP = 2000 + (dT * (int64_t)MS5611_C6) / (1<<23);
	OFF = (int64_t)MS5611_C2 * (1<<16) + ((int64_t)MS5611_C4 * dT) / (1<<7);
	SENS = (int64_t)MS5611_C1 * (1<<15) + ((int64_t)MS5611_C3 * dT) / (1<<8);
	*P = ((D1 * SENS) / (1<<21) - OFF) / (1<<15);
}

u8 MS5611_CRC4(u32 n_prom[])
{
	int cnt; // simple counter
	u32 n_rem; // crc reminder
	u32 crc_read; // original value of the crc
	u8 n_bit;
	n_rem = 0x00;
	crc_read = n_prom[7]; //save read CRC
	n_prom[7]=(0xFF00 & (n_prom[7])); //CRC byte is replaced by 0
	for(cnt = 0; cnt < 16; cnt++){ // operation is performed on bytes
		// choose LSB or MSB
		if((cnt & 0x01) == 1){
			n_rem ^= (u16)((n_prom[cnt >> 1]) & 0x00FF);
		}
		else{
			n_rem ^= (u16)(n_prom[cnt >> 1] >> 8);
		}
		for(n_bit = 8; n_bit > 0; n_bit--){
			if(n_rem & (0x8000)){
				n_rem = (n_rem << 1) ^ 0x3000;
			}
			else{
				n_rem = (n_rem << 1);
			}
		}
	}
	n_rem = (0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code
	n_prom[7] =crc_read; // restore the crc_read to its original place
	return (n_rem ^ 0x00);
}

void MS5611_Init(void)
{
	SPIx_Init(pMS5611);
	MS5611_Reset(pMS5611);
	Delay_Ms(1);
	MS5611_ReadPROM(pMS5611);
	Delay_Ms(1);
}
