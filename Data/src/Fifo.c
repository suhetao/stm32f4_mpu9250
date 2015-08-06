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

#include "Fifo.h"
#include "Memory.h"

void Fifo_Init(Fifo* fifo, u8 *buff, u16 len)
{
	fifo->Data = buff;
	fifo->Size = len;
	fifo->In = fifo->Out = 0;
}

u16 Fifo_Get(Fifo* fifo, u8 *buff, u16 len)
{
	u16 l;
	len = MIN(len, fifo->In - fifo->Out);  
	l = MIN(len, fifo->Size - (fifo->Out & (fifo->Size - 1)));  
	MemCpy(buff, fifo->Data + (fifo->Out & (fifo->Size - 1)), l);  
	MemCpy(buff + l, fifo->Data, len - l);  
	fifo->Out += len;  
	return len;  
}

void Fifo_Put(Fifo *fifo, u8 *buff, u16 len)
{
	u16 fixLen;
	//////////////////////////////////////////////////////////////////////////
	len = MIN(len, fifo->Size - fifo->In + fifo->Out);
	fixLen = MIN(len, fifo->Size - (fifo->In & (fifo->Size - 1)));
	MemCpy(fifo->Data + (fifo->In & (fifo->Size - 1)), buff, fixLen);  
	MemCpy(fifo->Data, buff + fixLen, len - fixLen);  		
	fifo->In += len;
}
