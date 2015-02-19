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

#include"stm32f4_delay.h"

static __IO u32 gMs_Ticks = 0;
static __IO u32 gMs_Counters = 0;

void Delay_Init()
{
	RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
	if(SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000u)){
		//Handle Error
		while (1);
	}
}

u32 Millis(void)
{
    return gMs_Ticks;
}

void Delay_Ms(u32 ms)
{
	gMs_Counters = ms;
	while(0 != gMs_Counters);
}

int Get_Ms(unsigned long *count)
{
	count[0] = gMs_Ticks;
	return 0;
}

void SysTick_Handler(void)
{
	gMs_Ticks++;
	if(0 != gMs_Counters){
		gMs_Counters--;
	}
}

