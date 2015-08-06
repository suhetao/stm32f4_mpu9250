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

#include "stm32f4_exti.h"

void EXTIx_Init(EXTI_Driver* EXTIx)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
  //Enable GPIO clocks
	EXTIx->GPIO_CLK(EXTIx->GPIO_Func, ENABLE);
  //Enable SYSCFG clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  //Configure GPIO pin as input floating
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = EXTIx->GPIO_Pin;
  GPIO_Init(EXTIx->Gpio, &GPIO_InitStructure);

  //Connect EXTI Line to GPIO Pin
	SYSCFG_EXTILineConfig(EXTIx->EXTI_PortSourceGPIO, EXTIx->EXTI_PinSource);
		
  //Configure EXTI line
  EXTI_Init(&EXTIx->EXIT_Init);
		
	//Enable and set EXTI Interrupt priority
  NVIC_Init(&EXTIx->NVIC_Init); 
}
