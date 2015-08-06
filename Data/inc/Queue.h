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

#ifndef _QUEUE_H
#define _QUEUE_H

#include "stm32f4xx.h"

#define MAX_MESSAGE_LEN (10)
#define MAX_QUEUE_LEN (32)
#define MAX_QUEUE_MASK (31)

typedef struct Buff_T
{
	s8 Buff[MAX_MESSAGE_LEN];
	u16 Len;
}Buff;

typedef struct QUEUE_T
{
	Buff Buffs[MAX_QUEUE_LEN];
	u16 Head;
	u16 Tail;
	u16 Size;
}Queue, *PQueue;

__inline s32 Queue_IsFull(PQueue queue)
{
	return (queue->Size == MAX_QUEUE_LEN);
}

__inline u16 Queue_Size(PQueue queue)
{
	return queue->Size;
}

__inline s32 Queue_IsEmpty(PQueue queue)
{
	return (queue->Size == 0);
}

s32 Queue_Enqueue(PQueue queue, s8* string, u16 len);
s32 Queue_Dequeue(PQueue queue, Buff* buff);

#endif
