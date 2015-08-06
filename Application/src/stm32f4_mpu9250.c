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
#include "stm32f4_exti.h"
#include "stm32f4_spi.h"
#include "stm32f4_delay.h"

//////////////////////////////////////////////////////////////////////////
//
static s16 MPU9250_AK8963_ASA[3] = {0, 0, 0};
//////////////////////////////////////////////////////////////////////////
//basic SPI driver for MPU9250
static SPI_Driver mMPU9250 = {
	SPI2, RCC_APB1PeriphClockCmd, RCC_APB1Periph_SPI2,
	GPIOB, RCC_AHB1PeriphClockCmd, RCC_AHB1Periph_GPIOB,
	GPIOB, RCC_AHB1PeriphClockCmd, RCC_AHB1Periph_GPIOB, GPIO_Pin_12,
	GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15,
	GPIO_PinSource13, GPIO_PinSource14, GPIO_PinSource15,	
#ifdef SPIx_USE_DMA

#endif
	{
		SPI_Direction_2Lines_FullDuplex, SPI_Mode_Master, SPI_DataSize_8b, 
		SPI_CPOL_High, SPI_CPHA_2Edge, SPI_NSS_Soft, SPI_BaudRatePrescaler_32,
		SPI_FirstBit_MSB, 7
	},
	GPIO_AF_SPI2
};
static SPI_Driver* pMPU9250 = &mMPU9250;

//
static EXTI_Driver mMPU9250INT= {
	GPIOB, RCC_AHB1PeriphClockCmd, RCC_AHB1Periph_GPIOB, GPIO_Pin_8, 
	EXTI_PortSourceGPIOB, EXTI_PinSource8,
	{
		EXTI_Line8, EXTI_Mode_Interrupt, EXTI_Trigger_Rising, ENABLE
	},
	{
		EXTI9_5_IRQn, 14, 0, ENABLE
	}
};
static EXTI_Driver* pMPU9250INT = &mMPU9250INT;
//////////////////////////////////////////////////////////////////////////
//
#define MPU9250_SPIx_SendByte(byte) SPIx_SendByte(pMPU9250, byte);
#define MPU9250_SPIx_SetDivisor(divisor) SPIx_SetDivisor(pMPU9250, divisor);

//////////////////////////////////////////////////////////////////////////
//init
void MPU9250_Init(void)
{
	u8 data = 0, state = 0;
	uint8_t response[3] = {0, 0, 0};
	//Lower level hardware Init
	SPIx_Init(pMPU9250);
	EXTIx_Init(pMPU9250INT);
	//////////////////////////////////////////////////////////////////////////
	//MPU9250 Reset
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_1, MPU9250_RESET);
	Delay_Ms(100);
	//MPU9250 Set Clock Source
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_1,  MPU9250_CLOCK_PLLGYROZ);
	Delay_Ms(1);
	//MPU9250 Set Interrupt
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_INT_PIN_CFG,  MPU9250_INT_ANYRD_2CLEAR);
	Delay_Ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_INT_ENABLE, ENABLE);
	Delay_Ms(1);
	//MPU9250 Set Sensors
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_PWR_MGMT_2, MPU9250_XYZ_GYRO & MPU9250_XYZ_ACCEL);
	Delay_Ms(1);
	//MPU9250 Set SampleRate
	//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_SMPLRT_DIV, SMPLRT_DIV);
	Delay_Ms(1);
	//MPU9250 Set Full Scale Gyro Range
	//Fchoice_b[1:0] = [00] enable DLPF
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_GYRO_CONFIG, (MPU9250_FSR_2000DPS << 3));
	Delay_Ms(1);
	//MPU9250 Set Full Scale Accel Range PS:2G
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG, (MPU9250_FSR_2G << 3));
	Delay_Ms(1);
	//MPU9250 Set Accel DLPF
	data = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG2);
	data |= MPU9250_ACCEL_DLPF_41HZ;
	Delay_Ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_ACCEL_CONFIG2, data);
	Delay_Ms(1);
	//MPU9250 Set Gyro DLPF
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_CONFIG, MPU9250_GYRO_DLPF_41HZ);
	Delay_Ms(1);
	//MPU9250 Set SPI Mode
	state = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL);
	Delay_Ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL, state | MPU9250_I2C_IF_DIS);
	Delay_Ms(1);
	state = MPU9250_SPIx_Read(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL);
	Delay_Ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_USER_CTRL, state | MPU9250_I2C_MST_EN);
	Delay_Ms(1);
	//////////////////////////////////////////////////////////////////////////
	//AK8963 Setup
	//reset AK8963
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL2, MPU9250_AK8963_CNTL2_SRST);
	Delay_Ms(2);

	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	Delay_Ms(1);
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);
	Delay_Ms(1);
	//
	//AK8963 get calibration data
	MPU9250_AK8963_SPIx_Reads(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_ASAX, 3, response);
	//AK8963_SENSITIVITY_SCALE_FACTOR
	//AK8963_ASA[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;
	MPU9250_AK8963_ASA[0] = (s16)(response[0]) + 128;
	MPU9250_AK8963_ASA[1] = (s16)(response[1]) + 128;
	MPU9250_AK8963_ASA[2] = (s16)(response[2]) + 128;
	Delay_Ms(1);
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	Delay_Ms(1);
	//
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_CTRL, 0x5D);
	Delay_Ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ);
	Delay_Ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_REG, MPU9250_AK8963_ST1);
	Delay_Ms(1);
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV0_CTRL, 0x88);
	Delay_Ms(1);
	//
	MPU9250_AK8963_SPIx_Write(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
	Delay_Ms(1);

	//
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 0x09);
	Delay_Ms(1);
	//
	MPU9250_SPIx_Write(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_DELAY_CTRL, 0x81);
	Delay_Ms(100);
}

int MPU9250_SPIx_Write(u8 addr, u8 reg_addr, u8 data){
	Chip_Select(pMPU9250);
	MPU9250_SPIx_SendByte(reg_addr);
	MPU9250_SPIx_SendByte(data);
	Chip_DeSelect(pMPU9250);
	return 0;
}

int MPU9250_SPIx_Writes(u8 addr, u8 reg_addr, u8 len, u8* data){
	u32 i = 0;
	Chip_Select(pMPU9250);
	MPU9250_SPIx_SendByte(reg_addr);
	while(i < len){
		MPU9250_SPIx_SendByte(data[i++]);
	}
	Chip_DeSelect(pMPU9250);
	return 0;
}

u8 MPU9250_SPIx_Read(u8 addr, u8 reg_addr)
{
	u8 dummy = 0;
	u8 data = 0;

	Chip_Select(pMPU9250);
	MPU9250_SPIx_SendByte(0x80 | reg_addr);
	data = MPU9250_SPIx_SendByte(dummy);
	Chip_DeSelect(pMPU9250);
	return data;
}

int MPU9250_SPIx_Reads(u8 addr, u8 reg_addr, u8 len, u8* data){
	u32 i = 0;
	u8 dummy = 0x00;

	Chip_Select(pMPU9250);
	MPU9250_SPIx_SendByte(MPU9250_I2C_READ | reg_addr);
	while(i < len){
		data[i++] = MPU9250_SPIx_SendByte(dummy);
	}
	Chip_DeSelect(pMPU9250);
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
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get9AxisRawData(short *accel, short * gyro, short * mag)
{
	u8 data[22];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_ACCEL_XOUT_H, 22, data);
	
	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];
	
	gyro[0] = (data[8] << 8) | data[9];
	gyro[1] = (data[10] << 8) | data[11];
	gyro[2] = (data[12] << 8) | data[13];

	if (!(data[14] & MPU9250_AK8963_DATA_READY) || (data[14] & MPU9250_AK8963_DATA_OVERRUN)){
		return;
	}
	if (data[21] & MPU9250_AK8963_OVERFLOW){
		return;
	}
	mag[0] = (data[16] << 8) | data[15];
	mag[1] = (data[18] << 8) | data[17];
	mag[2] = (data[20] << 8) | data[19];

	//ned x,y,z
	mag[0] = ((long)mag[0] * MPU9250_AK8963_ASA[0]) >> 8;
	mag[1] = ((long)mag[1] * MPU9250_AK8963_ASA[1]) >> 8;
	mag[2] = ((long)mag[2] * MPU9250_AK8963_ASA[2]) >> 8;
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get6AxisRawData(short *accel, short * gyro)
{
	u8 data[14];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_ACCEL_XOUT_H, 14, data);
	
	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];

	gyro[0] = (data[8] << 8) | data[9];
	gyro[1] = (data[10] << 8) | data[11];
	gyro[2] = (data[12] << 8) | data[13];
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisAccelRawData(short * accel)
{
	u8 data[6];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_ACCEL_XOUT_H, 6, data);

	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisGyroRawData(short * gyro)
{
	u8 data[6];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_GYRO_XOUT_H, 6, data);

	gyro[0] = (data[0] << 8) | data[1];
	gyro[1] = (data[2] << 8) | data[3];
	gyro[2] = (data[4] << 8) | data[5];
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisMagnetRawData(short *mag)
{
	u8 data[8];

	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_EXT_SENS_DATA_00, 8, data);
	if (!(data[0] & MPU9250_AK8963_DATA_READY) || (data[0] & MPU9250_AK8963_DATA_OVERRUN)){
		return;
	}
	if (data[7] & MPU9250_AK8963_OVERFLOW){
		return;
	}
	mag[0] = (data[2] << 8) | data[1];
	mag[1] = (data[4] << 8) | data[3];
	mag[2] = (data[6] << 8) | data[5];

	mag[0] = ((long)mag[0] * MPU9250_AK8963_ASA[0]) >> 8;
	mag[1] = ((long)mag[1] * MPU9250_AK8963_ASA[1]) >> 8;
	mag[2] = ((long)mag[2] * MPU9250_AK8963_ASA[2]) >> 8;
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_GetTemperatureRawData(long *temperature)
{
	u8 data[2];
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_TEMP_OUT_H, 2, data);
	temperature[0] = (((s16)data[0]) << 8) | data[1];
}

static vu8 MPU9250_IsNewData = 0;

int MPU9250_IsDataReady(void)
{
	int isNewData = MPU9250_IsNewData;
	MPU9250_IsNewData = 0;
	return isNewData;
}

//////////////////////////////////////////////////////////////////////////
//
void EXTI9_5_IRQHandler(void) 
{
	if(EXTI_GetITStatus(EXTI_Line8) != RESET){
    EXTI_ClearITPendingBit(EXTI_Line8);
		MPU9250_IsNewData = 1;
  }
}

