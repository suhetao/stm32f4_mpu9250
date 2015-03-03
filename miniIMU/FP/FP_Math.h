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

#ifndef _FP_FASTMATH_H_
#define _FP_FASTMATH_H_

#define FIXED_POINT
#define PRECISION 16

#define OPTIMIZE_SDIV

typedef int	Q16;
typedef int int32_t;
typedef unsigned int uint32_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef signed __int64 int64_t;
typedef unsigned __int64 uint64_t;

#define Q16_One 65536

//////////////////////////////////////////////////////////////////////////
//S16.16
#define FP_ADDS(a, b) ((a) + (b))
#define FP_SUBS(a, b) ((a) - (b))

#define FP_SMUL_FRAC(a, b, frac) \
	((int32_t)((int64_t)(a)*(int64_t)(b) >> (frac)))

#define FP_UMUL_FRAC(a, b, frac) \
	((uint32_t)((uint64_t)(a)*(uint64_t)(b) >> (frac)))

#define FP_SMUL(a, b) \
	((int32_t)((int64_t)(a)*(int64_t)(b) >> 16))
	
#define FP_UMUL(a, b) \
	((uint32_t)((uint64_t)(a)*(uint64_t)(b) >> 16))

Q16 FP_SDIV(Q16 a, Q16 b);

//////////////////////////////////////////////////////////////////////////
#define FP_ISQRT_UPDATE(est, mant2) \
	(((est) + ((est) >> 1)) - \
	FP_UMUL_FRAC(FP_UMUL_FRAC(mant2, est, 31), \
	FP_UMUL_FRAC(est, est, 31), 31))

Q16 FT_Q16(float f);

Q16 FP_Sqrt(Q16 xval, uint32_t frac);
Q16 FP_SqrtI(Q16 xval, int32_t frac);

__inline float Q16_FT(Q16 x)
{
	return ((float)x) / 65536.0f;
}
//////////////////////////////////////////////////////////////////////////
//translate from google's skia fixed-point dsp Library.
//
#define TO_FLOAT_DEGREE(x) (((float)(x)) * 0.000874264215087890625f)
#define Q16_TWOPI 411775
#define Q16_PI 205887
#define Q16_HALFPI 102944

Q16 FP_FastAsin(Q16 a);
Q16 FP_FastAtan2(Q16 y, Q16 x);

#endif
