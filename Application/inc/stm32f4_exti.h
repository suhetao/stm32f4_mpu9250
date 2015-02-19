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

#ifndef _STM32F4_EXTI_H
#define _STM32F4_EXTI_H

#include "stm32f4xx.h"

//Define the Interrupt pin
#define INTERRUPT_PIN                          GPIO_Pin_8   
#define INTERRUPT_GPIO_PORT                    GPIOB            
#define INTERRUPT_GPIO_CLK                     RCC_AHB1Periph_GPIOB
#define INTERRUPT_EXTI_LINE                    EXTI_Line8
#define INTERRUPT_EXTI_PORT_SOURCE             EXTI_PortSourceGPIOB
#define INTERRUPT_EXTI_PIN_SOURCE              GPIO_PinSource8
#define INTERRUPT_EDGE                         EXTI_Trigger_Rising 
#define INTERRUPT_EXTI_IRQN                    EXTI9_5_IRQn

#define INTERRUPT_EXTI_PREEMPTION_PRIORITY     14
#define INTERRUPT_EXTI_SUB_PRIORITY            0

void Interrupt_Init(void);
u8 Interrupt_GetState(void);

#endif
