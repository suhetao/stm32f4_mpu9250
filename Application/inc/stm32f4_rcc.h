#ifndef _STM32F4_RCC_H
#define _STM32F4_RCC_H

#include "stm32f4xx.h"

typedef struct PLL_PARAMS_T
{
	uint32_t PLLM;
	uint32_t PLLN;
	uint32_t PLLP;
	uint32_t PLLQ;
}
PLL_PARAMS;

typedef void (*RCC_AXXPeriphClockCmd)(uint32_t RCC_AXXPeriph, FunctionalState NewState);

void RCC_SystemCoreClockUpdate(PLL_PARAMS params);

#endif
