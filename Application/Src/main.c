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

#include "stm32f4_delay.h"
#include "stm32f4_mpu9250.h"
#include "stm32f4_rcc.h"
#include "stm32f4_exti.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define DEFAULT_MPU_HZ  (200)

//PLL_M PLL_N PLL_P PLL_Q
PLL_PARAMS gpFreq120M = {12, 240, 2, 5};
PLL_PARAMS gpFreq168M = {12, 336, 2, 7};

s32 gs32Result = 0;

int main(void)
{
	struct int_param_s pInitParam = {0};
	u8 u8AccelFsr = 0;
	u16 u16GyroRate = 0;
	u16 u16GyroFsr = 0;
	u16 u16DmpFeatures = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
		DMP_FEATURE_GYRO_CAL;
	
	s16 s16Gyro[3] = {0}, s16Accel[3] = {0}, s16Mag[3] = {0};
	s16 s16Sensors = 0;
	u8 u8More = 0;
	long lQuat[4] = {0};
	unsigned long ulTimeStamp = 0;

	//Reduced frequency
	//120 / 4 = 30Mhz APB1, 30/32 = 0.9375 MHz SPI Clock
	//1Mhz SPI Clock for read/write
	RCC_SystemCoreClockUpdate(gpFreq120M);
	Delay_Init();
	MPU9250_Init();

	//////////////////////////////////////////////////////////////////////////
	//Init DMP
	gs32Result = mpu_init(&pInitParam);
	gs32Result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	gs32Result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	gs32Result = mpu_set_sample_rate(DEFAULT_MPU_HZ);
	gs32Result = mpu_get_sample_rate(&u16GyroRate);
	gs32Result = mpu_get_gyro_fsr(&u16GyroFsr);
	gs32Result = mpu_get_accel_fsr(&u8AccelFsr);
	gs32Result = dmp_load_motion_driver_firmware();
	gs32Result = dmp_enable_feature(u16DmpFeatures);
	gs32Result = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	gs32Result = mpu_set_dmp_state(1);
	//////////////////////////////////////////////////////////////////////////
	//Recover frequency
	RCC_SystemCoreClockUpdate(gpFreq168M);
	Delay_Init();
	MPU9250_Init();
	//42Mhz APB1, 42/2 = 21 MHz SPI Clock
	MPU9250_SPIx_SetDivisor(2);
	Interrupt_Init();
	
	for(;;){
		if (Interrupt_GetState()){
 			gs32Result = dmp_read_fifo(s16Gyro, s16Accel, lQuat, &ulTimeStamp, &s16Sensors, &u8More);
			//because 20Mhz SPI Clock satisfy MPU9250 with read condition
			//so that you can't use I2C Master mode read/write from SPI at 20Mhz SPI Clock
			//and dmp fifo can't use for Magnetometer, but you can use this function below
			gs32Result = mpu_get_compass_reg(s16Mag, &ulTimeStamp);
			
			//todo
			//transmit the gyro, accel, mag, quat to anywhere
		}
	}
}
