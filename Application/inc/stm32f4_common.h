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

#ifndef _STM32F4_COMMON_H
#define _STM32F4_COMMON_H

#include "stm32f4xx.h"

__inline void FastMemCpy(uint8_t* pDest, uint8_t* pSrc, uint8_t length)
{
	uint32_t slice = length & 0x03;
	uint32_t intnum = length >> 2;
	
#if 1
	uint32_t * pintsrc = (uint32_t *)pSrc;
	uint32_t * pintdst = (uint32_t *)pDest;
		
	while(intnum--){
		*pintdst++ = *pintsrc++;
	}
	while(slice--){
		*((uint8_t *)pintdst++) = *((uint8_t *)pintsrc++);  
	}
#else
	while(intnum--){
		*pDest++ = *pSrc++;
		*pDest++ = *pSrc++;
		*pDest++ = *pSrc++;
		*pDest++ = *pSrc++;
	}
	while(slice--){
		*pDest++ = *pSrc++;
	}
#endif
}

#endif
