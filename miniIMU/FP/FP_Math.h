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

typedef int	Q16;
typedef unsigned int uint32_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

#define Q16_One 65536

typedef union {
	long i;
	float f;
}Long2Float;

//////////////////////////////////////////////////////////////////////////
//S16.16
__inline Q16 FT_Q16(float f)
{
	Long2Float l2f;
	int ival;
	int exponent;

	l2f.f = f;
	ival= (l2f.i & 0x07fffff) | 0x800000;
	exponent = 150 - ((l2f.i >> 23) & 0xff);
	if (exponent < 0)
		ival = ival << (-exponent - 16);
	else
		ival = ival >> (exponent - 16);

	if (l2f.i & 0x80000000){
		ival = -ival;
	}
	return ival;
}

__inline float Q16_FT(Q16 x)
{
	return ((float)x) / 65536.0f;
}

//////////////////////////////////////////////////////////////////////////
#ifndef FIXED_POINT
//32-bit
__inline Q16 FP_CMUL(Q16 a, Q16 b)
{
	short ah, bh;
	unsigned short al, bl;
	Q16 result;

	ah = a >> 16;
	al = a & 0x0000ffff;
	bh = b >> 16;
	bl = b & 0x0000ffff;
	result = (ah * bh)<<(16);
	result += (ah * bl);
	result += (al * bh);

	return result;
}

__inline Q16 FP_SDIV(Q16 a, Q16 b)
{
	int64_t result = (((int64_t)a) << 16) / ((int64_t)b);
	return (Q16)result;
}

__inline Q16 FP_UDIV(Q16 a, Q16 b)
{
	uint64_t result = (((uint64_t)a) << 16) / ((uint64_t)b);
	return (Q16)result;
}

#else
//////////////////////////////////////////////////////////////////////////
//cortex-m3's instruction assembly optimization
//signed 32bit * signed 32bit ->signe 64-bit >> 16 mul with frac
__inline Q16 FP_SMUL_FRAC(Q16 a, Q16 b, int frac)
{
	int __al, __ah, __frac;
	__asm{
		smull __al, __ah, a, b;
		subs __frac, frac, #0x20;
		bpl handle;
		rsb __frac, frac, #0x20;
		lsr __al, __al, frac;
		lsl __frac, __ah,  __frac;
		asr __ah, __ah, frac;
		orr __al, __al, __frac;
		b finish;
handle:
		asr __al, __ah, __frac;
		asr __ah, __ah, #31;
	}
finish:
	return __al;
}

//32bit * 32bit ->unsigned 64-bit >> 16 mul with frac
__inline Q16 FP_UMUL_FRAC(Q16 a, Q16 b, int frac)
{
	int __al, __ah, __frac;
	__asm{
		umull __al, __ah, a, b;
		subs __frac, frac, #0x20;
		bpl handle;
		rsb __frac, frac, #0x20;
		lsr __al, __al, frac;
		lsl __frac, __ah,  __frac;
		asr __ah, __ah, frac;
		orr __al, __al, __frac;
		b finish;
handle:
		asr __al, __ah, __frac;
		asr __ah, __ah, #31;
	}
finish:
	return __al;
}
//signed mul
__inline Q16 FP_SMUL(Q16 a, Q16 b)
{
	int __al, __ah, __r;
	__asm{
		smull __al, __ah, a, b;
		lsls __ah, __ah, #16;
		orr __r, __ah, __al, lsr #16;
	}
	return __r;
}
//unsigned mul
__inline Q16 FP_UMUL(Q16 a, Q16 b)
{
	int __al, __ah, __r;
	__asm{
		umull __al, __ah, a, b;
		lsls __ah, __ah, #16;
		orr __r, __ah, __al, lsr #16;
	}
	return __r;
}
//div64_32
__asm uint32_t UDIV64(uint32_t a, uint32_t b, uint32_t c)
{
	adds r0,r0,r0;
	adc r1,r1,r1;

	mov r3, #31;
loop
	cmp     r1,r2;
	subcs   r1,r1,r2;
	adcs    r0,r0,r0;
	adc     r1,r1,r1;
	sub r3, r3, #1;
	cmp r3, #0;
	bne loop
	cmp     r1,r2;
	subcs   r1,r1,r2;
	adcs    r0,r0,r0;
	bx      lr;
}
//div 32bit << 16 / 32bit
__inline Q16 FP_SDIV(Q16 a, Q16 b)
{
	int q;

	//different signs
	int sign = (a^b) < 0;
	uint32_t l,h;

	a = a < 0 ? -a : a;
	b = b < 0 ? -b : b;
	l = (a << 16);
	h = (a >> 16);

	q = UDIV64(l,h,b);

	if (sign){
		q = -q;
	}
	return q;
}

#endif

__inline Q16 FP_ADD(Q16 a, Q16 b)
{
	//no overflow detection
	int __r;
	__asm{
		adds __r, a, b
	}
	return __r;
}

__inline Q16 FP_SUB(Q16 a, Q16 b)
{
	//no overflow detection
	int __r;
	__asm{
		subs __r, a, b
	}
	return __r;
}

//////////////////////////////////////////////////////////////////////////
//for sqrt/sqrti
__inline FP_Norm(uint32_t* mant, int* expn, uint32_t xval, uint32_t frac)
{
	int nz__;
	__asm{
		clz nz__, xval
	}
	*mant   = xval << nz__;
	*expn = 31 - nz__ - frac;
}

#define FP_ISQRT_UPDATE(est, mant2)                            \
	(((est) + ((est) >> 1)) -                                  \
	(uint32_t)FP_UMUL_FRAC((uint32_t)FP_UMUL_FRAC(mant2, est, 31),     \
	(uint32_t)FP_UMUL_FRAC(est, est, 31), 31))

__inline uint32_t	FP_SqrtC(uint32_t mant, int expn, uint32_t iter)
{
	static const uint8_t ISQRT_LUT[256] =
	{150,149,148,147,146,145,144,143,142,141,140,139,138,137,136,135,
	134,133,132,131,131,130,129,128,127,126,125,124,124,123,122,121,
	120,119,119,118,117,116,115,114,114,113,112,111,110,110,109,108,
	107,107,106,105,104,104,103,102,101,101,100, 99, 98, 98, 97, 96,
	95, 95, 94, 93, 93, 92, 91, 91, 90, 89, 88, 88, 87, 86, 86, 85,
	84, 84, 83, 82, 82, 81, 80, 80, 79, 79, 78, 77, 77, 76, 75, 75,
	74, 74, 73, 72, 72, 71, 70, 70, 69, 69, 68, 67, 67, 66, 66, 65,
	65, 64, 63, 63, 62, 62, 61, 61, 60, 59, 59, 58, 58, 57, 57, 56,
	56, 55, 54, 54, 53, 53, 52, 52, 51, 51, 50, 50, 49, 49, 48, 48,
	47, 47, 46, 46, 45, 45, 44, 44, 43, 43, 42, 42, 41, 41, 40, 40,
	39, 39, 38, 38, 37, 37, 36, 36, 35, 35, 34, 34, 33, 33, 33, 32,
	32, 31, 31, 30, 30, 29, 29, 28, 28, 28, 27, 27, 26, 26, 25, 25,
	25, 24, 24, 23, 23, 22, 22, 22, 21, 21, 20, 20, 19, 19, 19, 18,
	18, 17, 17, 17, 16, 16, 15, 15, 15, 14, 14, 13, 13, 13, 12, 12,
	11, 11, 11, 10, 10,  9,  9,  9,  8,  8,  8,  7,  7,  6,  6,  6,
	5,  5,  5,  4,  4,  3,  3,  3,  2,  2,  2,  1,  1,  1,  0,  0};

	uint32_t m2 = mant / 2; //Half mantissa temporary
	uint32_t est;           //Inverse square root estimate

	//look up the initial guess from mantissa MSBs
	est = (uint32_t)(ISQRT_LUT[(mant >> 23) & 0xff] + 362) << 22;

	//iterate newton step: est = est*(3 - mant*est*est)/2
	switch (iter) {
		default:
		case 2:
			est = FP_ISQRT_UPDATE(est, m2);
			//fall through

		case 1:
			est = FP_ISQRT_UPDATE(est, m2);

		case 0:
			; //no action
	}

	//adjust estimate by 1/sqrt(2) if exponent is odd
	if (expn & 1) {
		est = (uint32_t)FP_UMUL_FRAC(est, 0xb504f334UL, 31);
	}

	return est;
}

__inline Q16 FP_Sqrt(Q16 xval, uint32_t frac)
{
    const uint8_t iter[] = {0, 1, 2, 2}; //result bits to #iter LUT
    uint32_t      est;                   //Estimated value
    uint32_t      mant;                  //Floating-point mantissa
    int           expn;                  //Floating-point exponent

    //Handle illegal values */
    if (xval < 0 || frac > 31) {
        return -1;
    }

    //Handle the trivial case */
    if (xval == 0) {
        return 0;
    }

    //Convert fixed-point number to floating-point with 32-bit mantissa */
    FP_Norm(&mant, &expn, xval, frac);

    //Call inverse square root core function */
    est = FP_SqrtC(mant, expn, iter[((expn >> 1) + frac) >> 3]);

    //Multiply estimation by mant to produce the square root */
    est = (uint32_t)FP_UMUL_FRAC(est, mant, 31);

    //The square root of the exponent */
    expn >>= 1;

    //Convert back to fixed-point */
    return est >> (31 - expn - frac);
}

__inline Q16 FP_SqrtI(Q16 xval, unsigned frac)
{
    uint32_t est;   //estimated value
    uint32_t mant;  //floating-point mantissa
    int      expn;  //floating-point exponent

    //handle illegal values
    if (xval <= 0 || frac > 31) {
        return -1;
    }

    //convert fixed-point number to floating-point with 32-bit mantissa
    FP_Norm(&mant, &expn, xval, frac);

    //call inverse square root core function
    est = FP_SqrtC(mant, expn, 2);

    //the square root of the exponent
    expn = -expn >> 1;

    //convert back to fixed-point
    return est >> (31 - expn - frac);
}
//////////////////////////////////////////////////////////////////////////
//stuff
__inline float FastSqrtI(float x)
{
	Long2Float l2f;
	float r;
	float y = 0.5f * x;

	l2f.f = y;
	l2f.i = 0x5f3759df - (l2f.i >> 1);
	r = l2f.f;
	r = r * (1.5f - y * r * r);
	//r = r * (1.5f - y * r * r);
	return r;
}

__inline float FastSqrt(float x)
{
	return x * FastSqrtI(x);
}

//translate from the DSP instruction of a DSP Library.
#define PI (3.1415926535897932384626433832795f)
#define PI_2 (1.5707963267948966192313216916398f)
#define PI_3 (1.0471975511965977461542144610932f)
#define PI_4 (0.78539816339744830961566084581988f)
#define PI_6 (0.52359877559829887307710723054658f)
#define TWO_MINUS_ROOT3 (0.26794919243112270647255365849413f)
#define SQRT3_MINUS_1 (0.73205080756887729352744634150587f)
#define SQRT3 (1.7320508075688772935274463415059f)
#define EPS_FLOAT (+3.452669830012e-4f)
//Coefficients used for atan/atan2
#define ATANP_COEF0 (-1.44008344874F)
#define ATANP_COEF1 (-7.20026848898e-1F)
#define ATANQ_COEF0 (+4.32025038919F)
#define ATANQ_COEF1 (+4.75222584599F)
//Coefficients used for asin/acos
#define ASINP_COEF1 (-2.7516555290596F)
#define ASINP_COEF2 (+2.9058762374859F)
#define ASINP_COEF3 (-5.9450144193246e-1F)
#define ASINQ_COEF0 (-1.6509933202424e+1F)
#define ASINQ_COEF1 (+2.4864728969164e+1F)
#define ASINQ_COEF2 (-1.0333867072113e+1F)

__inline float FastAsin(float x)
{
	float y, g;
	float num, den, result;
	long i;
	float sign = 1.0;

	y = x;
	if (y < (float)0.0){
		y = -y;
		sign = -sign;
	}

	if (y > (float)0.5){
		i = 1;
		if (y > (float)1.0){
			result = 0.0;
			return result;
		}    
		g = (1.0f - y) * 0.5f;
		y = -2.0f * FastSqrt(g);
	}
	else{
		i = 0;
		if (y < (float)EPS_FLOAT){
			result = y;
			if (sign < (float)0.0){
				result = -result;
			}
			return result;
		}
		g = y * y;
	}
	num = ((ASINP_COEF3 * g + ASINP_COEF2) * g + ASINP_COEF1) * g;
	den = ((g + ASINQ_COEF2) * g + ASINQ_COEF1) * g + ASINQ_COEF0;

	result = num / den;
	result = result * y + y;
	if (i == 1){
		result = result + (float)PI_2;
	}
	if (sign < (float)0.0){
		result = -result;
	}
	return result;
}

__inline float FastAtan2(float y, float x)
{
	float f, g;
	float num, den;
	float result;
	int n;

	static const float a[4] = {0, (float)PI_6, (float)PI_2, (float)PI_3};

	if (x == (float)0.0){
		if (y == (float)0.0){
			result = 0.0;
			return result;
		}

		result = (float)PI_2;
		if (y > (float)0.0){
			return result;
		}
		if (y < (float)0.0){
			result = -result;
			return result;
		}
	}
	n = 0;
	num = y;
	den = x;

	if (num < (float)0.0){
		num = -num;
	}
	if (den < (float)0.0){
		den = -den;
	}
	if (num > den){
		f = den;
		den = num;
		num = f;
		n = 2;
	}
	f = num / den;

	if (f > (float)TWO_MINUS_ROOT3){
		num = f * (float)SQRT3_MINUS_1 - 1.0f + f;
		den = (float)SQRT3 + f;
		f = num / den;
		n = n + 1;
	}

	g = f;
	if (g < (float)0.0){
		g = -g;
	}

	if (g < (float)EPS_FLOAT){
		result = f;
	}
	else{
		g = f * f;
		num = (ATANP_COEF1 * g + ATANP_COEF0) * g;
		den = (g + ATANQ_COEF1) * g + ATANQ_COEF0;
		result = num / den;
		result = result * f + f;
	}
	if (n > 1){
		result = -result;
	}
	result = result + a[n];

	if (x < (float)0.0){
		result = PI - result;
	}
	if (y < (float)0.0){
		result = -result;
	}
	return result;
}

#endif
