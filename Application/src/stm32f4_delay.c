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

//Cycles per microsecond
static __IO uint32_t us_ticks = 0;
static __IO uint32_t uptime_ticks = 0;
static __IO uint32_t cycle_ticks = 0;

void Delay_Init()
{
	RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
	us_ticks = RCC_Clocks.SYSCLK_Frequency / 1000000u;

	//enable DWT access
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	//enable the CPU cycle counter
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	//Reset counter
	DWT->CYCCNT = 0u;
	
	if(SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000u)){
		while (1); // Handle Error
	}
}

__inline uint32_t Millis(void)
{
    return uptime_ticks;
}

u32 Micros(void)
{
	register uint32_t old_cycle, cycle, timeMs;
	
	do{
		timeMs = __LDREXW(&uptime_ticks);
		cycle = DWT->CYCCNT;
		old_cycle = cycle_ticks;
	}
	while ( __STREXW( timeMs , &uptime_ticks ) );
	return (timeMs * 1000) + (cycle - old_cycle) / us_ticks;
}

void Delay_Ms(u32 ms)
{
	while (ms--){
		Delay_Us(1000);
	}
}

void Delay_Us(u32 us)
{
	uint32_t elapsed = 0;
	uint32_t elapsed_us = 0;
	uint32_t lastCount = DWT->CYCCNT;
	register uint32_t current_count = DWT->CYCCNT;
	
	for (;;) {
		current_count = DWT->CYCCNT;
		elapsed += current_count - lastCount;
		lastCount = current_count;
		
		elapsed_us = elapsed / us_ticks;
		if (elapsed_us >= us){
			break;
		}
		us -= elapsed_us;
		elapsed %= us_ticks;
	}
}

int Get_Ms(unsigned long *count)
{
	count[0] = uptime_ticks;
	return 0;
}

void SysTick_Handler(void)
{
	cycle_ticks = DWT->CYCCNT;
	uptime_ticks++;
}
