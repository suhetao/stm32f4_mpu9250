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
#include "stm32f4_mpu9250.h"

static __IO u8 gu8InterruptState = 0;

void Interrupt_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  //Enable MPU9250int GPIO clocks
  RCC_AHB1PeriphClockCmd(INTERRUPT_GPIO_CLK, ENABLE);
  //Enable SYSCFG clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  //Configure MPU9250int pin as input floating
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = INTERRUPT_PIN;
  GPIO_Init(INTERRUPT_GPIO_PORT, &GPIO_InitStructure);

  //Connect MPU9250int EXTI Line to MPU9250int GPIO Pin
	SYSCFG_EXTILineConfig(INTERRUPT_EXTI_PORT_SOURCE, INTERRUPT_EXTI_PIN_SOURCE);
		
  //Configure MPU9250int EXTI line
  EXTI_InitStructure.EXTI_Line = INTERRUPT_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
		
	//Enable and set MPU9250int EXTI Interrupt priority
  NVIC_InitStructure.NVIC_IRQChannel = INTERRUPT_EXTI_IRQN;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INTERRUPT_EXTI_PREEMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = INTERRUPT_EXTI_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
}

void EXTI9_5_IRQHandler(void) 
{
	gu8InterruptState = 1;
	EXTI_ClearITPendingBit(EXTI_Line8);
}

u8 Interrupt_GetState(void){
	u8 u8State = gu8InterruptState;
	gu8InterruptState = 0;
	return u8State;
}
