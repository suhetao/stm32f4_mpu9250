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
	{
		SPI_Direction_2Lines_FullDuplex, SPI_Mode_Master, SPI_DataSize_8b, 
		SPI_CPOL_High, SPI_CPHA_2Edge, SPI_NSS_Soft, SPI_BaudRatePrescaler_4,
		SPI_FirstBit_MSB, 7
	},
	GPIO_AF_SPI1
};
static SPI_Driver* pMS5611 = &mMS5611;

__IO u16 MS5611_C1 = 0, MS5611_C2 = 0, MS5611_C3 = 0, MS5611_C4 = 0, MS5611_C5 = 0, MS5611_C6 = 0;

void MS5611_Reset(SPI_Driver *MS5611)
{
	// Chip Select low 
	Chip_Select(MS5611);
	SPIx_SendByte(MS5611, MS5611_RESET);
	Delay_Ms(3); //2.8ms reload from datasheet
	// Chip Select high
	Chip_DeSelect(MS5611);
}

u16 MS5611_SPIx_ReadWord(SPI_Driver *MS5611, u8 addr)
{
	u8 data[2] = {0};
	u16 value = 0;

	// Chip Select low 
	Chip_Select(MS5611);
	SPIx_SendByte(MS5611, addr);
	data[0] = SPIx_SendByte(MS5611, MS5611_READ);
	data[1] = SPIx_SendByte(MS5611, MS5611_READ);
	// Chip Select high
	Chip_DeSelect(MS5611);

	value = data[0] << 8 | data[1];
	return value;
}

void MS5611_SPIx_ReadADC(SPI_Driver *MS5611, u8 osr, u32* value)
{
	u8 data[3] = {0};

	// Chip Select low 
	Chip_Select(MS5611);
	SPIx_SendByte(MS5611, osr);
	// Chip Select high
	Chip_DeSelect(MS5611);

	Delay_Ms(1);

	// Chip Select low 
	Chip_Select(MS5611);
	SPIx_SendByte(MS5611, MS5611_READ_ADC);
	data[0] = SPIx_SendByte(MS5611, MS5611_READ_ADC);
	data[1] = SPIx_SendByte(MS5611, MS5611_READ_ADC);
	data[2] = SPIx_SendByte(MS5611, MS5611_READ_ADC);
	// Chip Select high
	Chip_DeSelect(MS5611);

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
typedef uint64_t u64;
typedef int64_t s64;

void MS5611_GetTemperatureAndPressure(s32* T, s32 *P)
{
	u32 D1, D2;
	s32 dT, TEMP, T2 = 0;
	s64 OFF, SENS, OFF2 = 0, SENS2 = 0;
	s32 lowTEMP, verylowTemp;
	
	MS5611_SPIx_ReadADC(pMS5611, D1_OSR_256, &D1);
	MS5611_SPIx_ReadADC(pMS5611, D2_OSR_256, &D2);
	//////////////////////////////////////////////////////////////////////////
	//
	dT = D2 - ((u32)MS5611_C5 << 8);
	TEMP = 2000 + (((s64)dT * MS5611_C6) >> 23);
	OFF = ((u32)MS5611_C2 << 16) + ((MS5611_C4 * (s64)dT) >> 7);
	SENS = ((u32)MS5611_C1 << 15) + ((MS5611_C3 * (s64)dT) >> 8);
	//
	*T = TEMP;
	//////////////////////////////////////////////////////////////////////////
	//second order temperature compensation
	if(TEMP < 2000){
		T2 = (s64)((s64)dT * (s64)dT) >> 31;
		lowTEMP = TEMP - 2000;
		lowTEMP *= lowTEMP;
		OFF2 = (5 * lowTEMP) >> 1;
		SENS2 = (5 * lowTEMP) >> 2;
		if(TEMP < -1500){
			verylowTemp = TEMP + 1500;
			verylowTemp *= verylowTemp;
			OFF2 = OFF2 + 7 * verylowTemp;
			SENS2 = SENS2 + ((11 * verylowTemp) >> 1);
		}
		//
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
		*T = TEMP - T2;
	}
	//////////////////////////////////////////////////////////////////////////
	*P = ((((u64)D1 * SENS) >> 21) - OFF) >> 15;
}

void MS5611_Cal(s32* T, s32 *P)
{
	u32 D1, D2;
	s32 dT, TEMP, T2 = 0;
	s64 OFF, SENS, OFF2 = 0, SENS2 = 0;
	s32 lowTEMP, verylowTemp;
	
	MS5611_SPIx_ReadADC(pMS5611, D1_OSR_256, &D1);
	MS5611_SPIx_ReadADC(pMS5611, D2_OSR_256, &D2);
	//////////////////////////////////////////////////////////////////////////
	//
	dT = D2 - ((u32)MS5611_C5 << 8);
	TEMP = 2000 + (((s64)dT * MS5611_C6) >> 23);
	OFF = ((u32)MS5611_C2 << 16) + ((MS5611_C4 * (s64)dT) >> 7);
	SENS = ((u32)MS5611_C1 << 15) + ((MS5611_C3 * (s64)dT) >> 8);
	//
	*T = TEMP;
	//////////////////////////////////////////////////////////////////////////
	//second order temperature compensation
	if(TEMP < 2000){
		T2 = (s64)((s64)dT * (s64)dT) >> 31;
		lowTEMP = TEMP - 2000;
		lowTEMP *= lowTEMP;
		OFF2 = (5 * lowTEMP) >> 1;
		SENS2 = (5 * lowTEMP) >> 2;
		if(TEMP < -1500){
			verylowTemp = TEMP + 1500;
			verylowTemp *= verylowTemp;
			OFF2 = OFF2 + 7 * verylowTemp;
			SENS2 = SENS2 + ((11 * verylowTemp) >> 1);
		}
		//
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
		*T = TEMP - T2;
	}
	//////////////////////////////////////////////////////////////////////////
	*P = ((((u64)D1 * SENS) >> 21) - OFF) >> 15;
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
