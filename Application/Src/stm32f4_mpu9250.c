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

#include "stm32f4_mpu9250.h"
#include "stm32f4_delay.h"

void MPU9250_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	MPU9250_SPIx_CLK_INIT(MPU9250_SPIx_CLK, ENABLE);
	//Enable SCK, MOSI and MISO GPIO clocks
	MPU9250_SPIx_GPIO_CLK_INIT(MPU9250_SPIx_CS_GPIO_CLK
		| MPU9250_SPIx_SCK_GPIO_CLK
		| MPU9250_SPIx_MISO_GPIO_CLK
		| MPU9250_SPIx_MOSI_GPIO_CLK, ENABLE);

	//GPIO Deinitialisation
	GPIO_DeInit(MPU9250_SPIx_SCK_GPIO_PORT);
	GPIO_DeInit(MPU9250_SPIx_MISO_GPIO_PORT);
	GPIO_DeInit(MPU9250_SPIx_MOSI_GPIO_PORT);

	//Connect SPI pins to AF5
	GPIO_PinAFConfig(MPU9250_SPIx_SCK_GPIO_PORT,
		MPU9250_SPIx_SCK_SOURCE, MPU9250_SPIx_SCK_AF);
	GPIO_PinAFConfig(MPU9250_SPIx_MISO_GPIO_PORT,
		MPU9250_SPIx_MISO_SOURCE, MPU9250_SPIx_MISO_AF);    
	GPIO_PinAFConfig(MPU9250_SPIx_MOSI_GPIO_PORT,
		MPU9250_SPIx_MOSI_SOURCE, MPU9250_SPIx_MOSI_AF);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	//SPI SCK/MISO/MOSI pin configuration
	GPIO_InitStructure.GPIO_Pin = MPU9250_SPIx_SCK_PIN
		| MPU9250_SPIx_MOSI_PIN
		| MPU9250_SPIx_MISO_PIN;
	GPIO_Init(MPU9250_SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);

	//Configure GPIO PIN for Chip select
	GPIO_InitStructure.GPIO_Pin = MPU9250_SPIx_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MPU9250_SPIx_CS_GPIO_PORT, &GPIO_InitStructure);

	//
	MPU9250_Deselect();

	//SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(MPU9250_SPIx);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // 30/32 = 0.9375 MHz SPI Clock
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(MPU9250_SPIx, &SPI_InitStructure);

	SPI_CalculateCRC(MPU9250_SPIx, DISABLE);

	//Enable SPIx
	SPI_Cmd(MPU9250_SPIx, ENABLE);
	while (SPI_I2S_GetFlagStatus(MPU9250_SPIx, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_ReceiveData(MPU9250_SPIx);
}

u8 MPU9250_SPIx_SendByte(u8 data)
{
	while (SPI_I2S_GetFlagStatus(MPU9250_SPIx, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(MPU9250_SPIx, data);
	while (SPI_I2S_GetFlagStatus(MPU9250_SPIx, SPI_I2S_FLAG_RXNE) == RESET);
	return((uint8_t)SPI_I2S_ReceiveData(MPU9250_SPIx));
}

void MPU9250_SPIx_SetDivisor(u16 data)
{
	uint16_t tempRegister;
	SPI_Cmd(MPU9250_SPIx, DISABLE);

	tempRegister = MPU9250_SPIx->CR1;

	switch (data)
	{
	case 2:
		tempRegister &= MPU9250_SPIx_BR_CLEAR_MASK;
		tempRegister |= SPI_BaudRatePrescaler_2;
		break;
	case 4:
		tempRegister &= MPU9250_SPIx_BR_CLEAR_MASK;
		tempRegister |= SPI_BaudRatePrescaler_4;
		break;
	case 8:
		tempRegister &= MPU9250_SPIx_BR_CLEAR_MASK;
		tempRegister |= SPI_BaudRatePrescaler_8;
		break;
	case 16:
		tempRegister &= MPU9250_SPIx_BR_CLEAR_MASK;
		tempRegister |= SPI_BaudRatePrescaler_16;
		break;
	case 32:
		tempRegister &= MPU9250_SPIx_BR_CLEAR_MASK;
		tempRegister |= SPI_BaudRatePrescaler_32;
		break;
	case 64:
		tempRegister &= MPU9250_SPIx_BR_CLEAR_MASK;
		tempRegister |= SPI_BaudRatePrescaler_64;
		break;
	case 128:
		tempRegister &= MPU9250_SPIx_BR_CLEAR_MASK;
		tempRegister |= SPI_BaudRatePrescaler_128;
		break;
	case 256:
		tempRegister &= MPU9250_SPIx_BR_CLEAR_MASK;
		tempRegister |= SPI_BaudRatePrescaler_256;
		break;
	}

	MPU9250_SPIx->CR1 = tempRegister;
	SPI_Cmd(MPU9250_SPIx, ENABLE);
}

int MPU9250_SPIx_Write(u8 addr, u8 reg_addr, u8 data){
	MPU9250_Select();
	MPU9250_SPIx_SendByte(reg_addr);
	MPU9250_SPIx_SendByte(data);
	MPU9250_Deselect();
	return 0;
}

int MPU9250_SPIx_Writes(u8 addr, u8 reg_addr, u8 len, u8* data){
	u32 i = 0;
	MPU9250_Select();
	MPU9250_SPIx_SendByte(reg_addr);
	while(i < len){
		MPU9250_SPIx_SendByte(data[i++]);
	}
	MPU9250_Deselect();
	return 0;
}

u8 MPU9250_SPIx_Read(u8 addr, u8 reg_addr)
{
	u8 dummy = 0;
	u8 data = 0;

	MPU9250_Select();
	MPU9250_SPIx_SendByte(0x80 | reg_addr);
	data = MPU9250_SPIx_SendByte(dummy);
	MPU9250_Deselect();
	return data;
}

int MPU9250_SPIx_Reads(u8 addr, u8 reg_addr, u8 len, u8* data){
	u32 i = 0;
	u8 dummy = 0x00;

	MPU9250_Select();
	MPU9250_SPIx_SendByte(MPU9250_I2C_READ | reg_addr);
	while(i < len){
		data[i++] = MPU9250_SPIx_SendByte(dummy);
	}
	MPU9250_Deselect();
	return 0;
}

int MPU9250_AK8963_SPIx_Read(u8 akm_addr, u8 reg_addr, u8* data) {
	u8 status = 0;
	u32 timeout = 0;

	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &reg_addr);
	Delay_Ms(1);
	reg_addr = akm_addr | MPU9250_I2C_READ;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &reg_addr);
	Delay_Ms(1);
	reg_addr = MPU9250_I2C_SLV4_EN;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &reg_addr);
	Delay_Ms(1);

	do {
		if (timeout++ > 50){
			return -2;
		}
		MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
		Delay_Ms(1);
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DI, 1, data);
	return 0;
}

int MPU9250_AK8963_SPIx_Reads(u8 akm_addr, u8 reg_addr, u8 len, u8* data){
	u8 index = 0;
	u8 status = 0;
	u32 timeout = 0;
	u8 tmp = 0;

	tmp = akm_addr | MPU9250_I2C_READ;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	Delay_Ms(1);
	while(index < len){
		tmp = reg_addr + index;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
		Delay_Ms(1);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		Delay_Ms(1);

		do {
			if (timeout++ > 50){
				return -2;
			}
			MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
			Delay_Ms(2);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DI, 1, data + index);
		Delay_Ms(1);
		index++;
	}
	return 0;
}

int MPU9250_AK8963_SPIx_Write(u8 akm_addr, u8 reg_addr, u8 data)
{
	u32 timeout = 0;
	uint8_t status = 0;
	u8 tmp = 0;

	tmp = akm_addr;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	Delay_Ms(1);
	tmp = reg_addr;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
	Delay_Ms(1);
	tmp = data;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, &tmp);
	Delay_Ms(1);
	tmp = MPU9250_I2C_SLV4_EN;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
	Delay_Ms(1);

	do {
		if (timeout++ > 50)
			return -2;

		MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
		Delay_Ms(1);
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	if (status & MPU9250_I2C_SLV4_NACK)
		return -3;
	return 0;
}

int MPU9250_AK8963_SPIx_Writes(u8 akm_addr, u8 reg_addr, u8 len, u8* data)
{
	u32 timeout = 0;
	uint8_t status = 0;
	u8 tmp = 0;
	u8 index = 0;

	tmp = akm_addr;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	Delay_Ms(1);

	while(index < len){
		tmp = reg_addr + index;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
		Delay_Ms(1);
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, data + index);
		Delay_Ms(1);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		Delay_Ms(1);

		do {
			if (timeout++ > 50)
				return -2;
			MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
			Delay_Ms(1);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		if (status & MPU9250_I2C_SLV4_NACK)
			return -3;
		index++;
	}
	return 0;
}

