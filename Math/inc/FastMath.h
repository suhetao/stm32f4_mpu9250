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

#ifndef _FASTMATH_H_
#define _FASTMATH_H_

#include "stm32f4xx.h"
#include "arm_math.h"

typedef union {
	int i;
	float32_t f;
}Int2Float;

typedef union {
	long i;
	float32_t f;
}Long2Float;

__inline float32_t arm_fabs_f32(float32_t x)
{
	Long2Float l2f;
	l2f.f = x;
	l2f.i &= ~0x80000000;
	return l2f.f;
}

__inline float32_t FastInvSqrt(float32_t x)
{
	Long2Float l2f;
	float32_t r;
	float32_t y = 0.5f * x;
	
	l2f.f = y;
	l2f.i = 0x5f3759df - (l2f.i >> 1);
	r = l2f.f;
	r = r * (1.5f - y * r * r);
	//r = r * (1.5f - y * r * r);
	return r;
}

__inline float32_t FastSqrt(float32_t x)
{
	return x * FastInvSqrt(x);
}

float32_t FastLn(float32_t x);
float32_t FastAtan2(float32_t y, float32_t x);
float32_t FastAsin(float32_t x);

#endif
