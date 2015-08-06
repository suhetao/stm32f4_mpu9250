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

#ifndef _STM32F4_UBLOX_H
#define _STM32F4_UBLOX_H

#include "stm32f4xx.h"
#include "Nema.h"

#define UBLOX_DEFAULT_BAUDRATE (9600)
#define UBLOX_DEFAULT_TX_BUFFERSIZE (64)
#define UBLOX_DEFAULT_RX_BUFFERSIZE (512)
#define UBLOX_DEFAULT_PARSER_MAXSIZE (128)

typedef struct UBLOX_PARSERBUFF
{
	s8 *Data;
	u16 Size;
	u16 Need;
	u16 Left;
}Ublox_ParserBuff;

void Ublox_Init(void);
void Ublox_GetMessage(void);
void Ublox_GetPostion(double *x, double *y, double *z);

#endif
