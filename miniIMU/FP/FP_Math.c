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

#include "FP_Math.h"

typedef union {
	long i;
	float f;
}L2F;

//////////////////////////////////////////////////////////////////////////
//S16.16
Q16 FT_Q16(float f)
{
	L2F l2f;
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

#ifdef OPTIMIZE_SDIV
//translate from arm-gcc build-in toolchain
//cortex-m3's instruction assembly optimization
//div signed 32bit << 16 / signed 32bit = signed 32bit
__asm Q16 FP_SDIV(Q16 num, Q16 den)
{
	//num r0, den r1, mod r2, cnt r3, quo r4, sign r12
	//set sign and ensure numerator and denominator are positive
	cmp r1, #0; //exceptioin if den == zero
	beq div0; //
	eor r12, r0, r1; //sign = num ^ den
	rsbmi r1, r1, #0; //den = -den if den < 0
	subs r2, r1, #1; //mod = den - 1
	beq div1; //return if den == 1
	movs r3, r0; //num = -num if num < 0
	rsbmi r0, r0, #0;
	//skip if deniminator >= numerator
	movs r3, r0, lsr #16; //return if den >= num << 16
	bne cont;
	cmp r1, r0, lsl #16;
	bhs numLeDen;
cont
	//test if denominator is a power of two
	tst r1, r2; //if(den & (den - 1) == 0)
	beq powerOf2; //den is power of 2
	stmfd sp!, {r4};// push r4 (quo) onto the stack
	//count leading zeros
	clz r2, r1;
	clz r3, r0;
	sub r2, r2, r3;

	rsb r3, r2, #31;
	and r3, r3, #30;
	rsb r2, r3, #32;
	mov r2, r0, lsr r2;
	mov r0, r0, lsl r3;

	rsb r3, r3, #32 + 16

	mov r4, #0
	rsb r1, r1, #0; //negate den for divide loop

inner_loop
	adds r0, r0, r0; //start: num = mod:num / den
	adcs r2, r1, r2, lsl #1;
	subcc r2, r2, r1;
	adc r4, r4, r4;	
	subs r3, r3, #1;
	bne inner_loop

	// negate quotient if signed
	cmp r12, #0; //negate quotient if sign < 0
	mov r0, r4
	rsbmi r0, r0, #0;
	ldmfd sp!, {r4}; // pop r4 (quo) off the stack
	bx lr; //return
	//divide by zero handler
div0
	mov r0, #0
	bx lr; //return
	//divide by one handler
div1
	cmp r12, #0
	mov r0, r0, lsl #16//mov r0, r0, asl #16
	rsbmi r0, r0, #0
	bx lr; //return
	//numerator less than or equal to denominator handler
numLeDen
	mov r0, #0; //quotient = 0 if num < den
	moveq r0, r12, asr #31; //negate quotient if sign < 0
	orreq r0, r0, #1; //quotient = 1 if num == den
	bx lr; //return
	//power of two handler
powerOf2
	clz r3, r1;
	rsb r3, r3, #31;
	rsb r2, r3, #32
	mov r1, r0, lsr #16; //den:num = num << 16
	mov r0, r0, lsl #16
	mov r0, r0, lsr r3; //num = num >> cnt | den << mod
	//orr r0, r0, r1, lsl r2
	lsl r1, r1, r2;
	orr r0, r0, r1

	cmp r12, #0
	rsbmi r0, r0, #0; //negate quotient if sign < 0"
	bx lr
}
#else
//udiv64_32
__asm uint32_t UDIV64(uint32_t a, uint32_t b, uint32_t c)
{
	adds r0, r0, r0;
	adc r1, r1, r1;

	mov r3, #31;
loop
	cmp r1, r2;
	subcs r1, r1, r2;
	adcs r0, r0, r0;
	adc r1, r1, r1;
	sub r3, r3, #1;
	cmp r3, #0;
	bne loop
	cmp r1, r2;
	subcs r1, r1, r2;
	adcs r0, r0, r0;
	bx lr;
}

//div 32bit << 16 / 32bit
Q16 FP_SDIV(Q16 a, Q16 b)
{
	int q;

	//different signs
	int sign = (a^b) < 0;
	uint32_t l,h;

	a = a < 0 ? -a : a;
	b = b < 0 ? -b : b;
	l = (a << 16);
	h = (a >> 16);

	q = UDIV64(l, h, b);

	if (sign){
		q = -q;
	}
	return q;
}
#endif

static const uint8_t ISQRT_LUT[256] ={
	150,149,148,147,146,145,144,143,142,141,140,139,138,137,136,135,
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
	5,  5,  5,  4,  4,  3,  3,  3,  2,  2,  2,  1,  1,  1,  0,  0
};

uint32_t FP_SqrtC(uint32_t mant, int expn, uint32_t iter)
{
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

__asm uint8_t CLZ(uint32_t value)
{
	mov r1, r0;
  clz r0, r1
	bx lr; //return
}

Q16 FP_Sqrt(Q16 xval, uint32_t frac)
{
	const uint8_t iter[] = {0, 1, 2, 2}; //result bits to #iter LUT
	uint32_t est; //estimated value
	uint32_t mant; //floating-point mantissa
	int expn; //floating-point exponent
	int nz;

	//handle illegal values
	if (xval < 0 || frac > 31) {
		return -1;
	}

	//handle the trivial case
	if (xval == 0) {
		return 0;
	}

	//convert fixed-point number to floating-point with 32-bit mantissa
	nz = CLZ(xval);
	mant = xval << nz;
	expn = 31 - nz - frac;

	//call inverse square root core function
	est = FP_SqrtC(mant, expn, iter[((expn >> 1) + frac) >> 3]);

	//multiply estimation by mant to produce the square root
	est = FP_UMUL_FRAC(est, mant, 31);

	//the square root of the exponent
	expn >>= 1;

	//convert back to fixed-point
	return est >> (31 - expn - frac);
}

Q16 FP_SqrtI(Q16 xval, int32_t frac)
{
	uint32_t est;   //estimated value
	uint32_t mant;  //floating-point mantissa
	int expn;  //floating-point exponent
	int nz;

	//handle illegal values
	if (xval <= 0 || frac > 31) {
		return -1;
	}

	//convert fixed-point number to floating-point with 32-bit mantissa
	nz = CLZ(xval);
	mant = xval << nz;
	expn = 31 - nz - frac;

	//call inverse square root core function
	est = FP_SqrtC(mant, expn, 2);

	//the square root of the exponent
	expn = -expn >> 1;

	//convert back to fixed-point
	return est >> (31 - expn - frac);
}

//////////////////////////////////////////////////////////////////////////
//translate from google's skia fixed-point dsp Library.
//
const int ATAN_TABLE[] = {
	0x20000000, 0x12E4051D, 0x9FB385B, 0x51111D4, 0x28B0D43, 0x145D7E1, 0xA2F61E, 0x517C55,
	0x28BE53, 0x145F2E, 0xA2F98, 0x517CC, 0x28BE6, 0x145F3, 0xA2F9, 0x517C,
	0x28BE, 0x145F, 0xA2F, 0x517, 0x28B, 0x145, 0xA2, 0x51,
	0x28, 0x14, 0xA, 0x5, 0x2, 0x1
};

__asm Q16 FP_ScaleBack(Q16 a, Q16 s)
{
	smull r3, r4, r0, r1;
	mov r0, r4;
	bx lr; //return
}

Q16 FP_FastAsin(Q16 a)
{
	int x, y, z;
	int x1, y1, z1, t, tan;
	int sign = a >> 31;
	const int* tanPtr = ATAN_TABLE;
	
	if(a < 0){
		z = -a;
	}
	else{
		z = a;
	}
	if (z >= Q16_One){
		return (Q16_HALFPI ^ (~sign)) - sign;
	}
	x = 0x18bde0bb; //gain parameter 0.607252935
	y = 0;
	z *= 0x28be;
	
	z1 = 0;
	t = 0;
	
	do {
		x1 = y >> t;
		y1 = x >> t;
		tan = *tanPtr++;
		if (y < z) {
			x -= x1;
			y += y1;
			z1 -= tan;
		} else {
			x += x1;
			y -= y1;
			z1 += tan;
		}
	} while (++t < 16); // 30
	//scale back into the scalar space (0x100000000/0x28be = #0x6488d)
	z = FP_ScaleBack(z1, 0x6488d);
	
	z = (z ^ (~sign)) - sign;
	return z;
}

Q16 FP_FastAtan2(Q16 y, Q16 x)
{
	int xsign, rsign;
	int pi, result;
	int z, t;
	int x1, y1, tan;
	
	const int* tanPtr = ATAN_TABLE;
	if ((x | y) == 0){
		return 0;
	}
	xsign = x >> 31;
	if(x < 0){
		x = -x;
	}
	
	z = 0;
	t = 0;
	do {
		x1 = y >> t;
		y1 = x >> t;
		tan = *tanPtr++;
		if (y < 0) {
			x -= x1;
			y += y1;
			z -= tan;
		}
		else{
			x += x1;
			y -= y1;
			z += tan;
		}
	} while (++t < 16); // 30
	//cortex-m3's instruction assembly optimization
	//scale back into the scalar space (0x100000000/0x28be = #0x6488d)
	result = FP_ScaleBack(z, 0x6488d);
	if (xsign) {
		rsign = result >> 31;
		if (y == 0){
			rsign = 0;
		}
		pi = (Q16_PI ^ (~rsign)) - rsign;
		result = pi - result;
	}
	return result;
}
