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

#include "Queue.h"
#include "Memory.h"

s32 Queue_Enqueue(PQueue queue, s8* string, u16 len)
{
	if(Queue_IsFull(queue)){
		return -1;
	}
	queue->Size++;
	//////////////////////////////////////////////////////////////////////////
	queue->Buffs[queue->Tail].Len = len;
	MemCpy((u8*)queue->Buffs[queue->Tail].Buff, (u8*)string, len);
	//////////////////////////////////////////////////////////////////////////
	queue->Tail++;
	queue->Tail &= MAX_QUEUE_MASK;
	return 0;
}

s32 Queue_Dequeue(PQueue queue, Buff* buff)
{
	if(Queue_IsEmpty(queue)){
		return -1;
	}
	queue->Size--;
	//////////////////////////////////////////////////////////////////////////
	*buff = queue->Buffs[queue->Head];
	//////////////////////////////////////////////////////////////////////////
	queue->Head++;
	queue->Head &= MAX_QUEUE_MASK;
	return 0;
}
