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

#ifndef _DOUBLE_H
#define _DOUBLE_H

//////////////////////////////////////////////////////////////////////////
//algorithm from the book
//<<Fast and Accurate Finite-Element Multigrid Solvers for PDE Simulations on GPU Clusters>>
//References:
//Emulated double precision Double single routine from nvidia developer zone
//////////////////////////////////////////////////////////////////////////
//define my own double struct
typedef struct
{
	float hi;
	float lo;
} Double;
//////////////////////////////////////////////////////////////////////////
//transmit double precision float to my own Double
__inline Double intToDouble(int A)
{
	Double B;

	B.hi = (float)A;
	B.lo = 0.0f;
	
	return B;
}
//
__inline Double floatToDouble(float A)
{
	Double B;

	B.hi = A;
	B.lo = 0.0f;
	
	return B;
}
//
__inline Double doubleToDouble(double A)
{
	Double B;

	B.hi = (float)A;
	B.lo = (float)(A - (double)B.hi);
	
	return B;
}
//transmit my own Double to double precision float
__inline double DoubleTodouble(Double B)
{
	double A;

	A = B.hi;
	A += B.lo;

	return A;
}

//addition: Double + Double
__inline Double DoubleAdd(Double A, Double B)
{
	Double C;
	float t1, t2, e;

	//Compute high order sum and error
	t1 = A.hi + B.hi;
	e = t1 - A.hi;
	//Compute low order term, including error and overflows
	t2 = ((B.hi - e) + ( A.hi - (t1 - e))) + A.lo + B.lo;
	//Normalise to get final result
	C.hi = t1 + t2;
	C.lo = t2 -(C.hi - t1);

	return C;
}

//Subtraction: Double - Double
__inline Double DoubleSub(Double A, Double B)
{
	Double C;
	float t1, t2, e;

	//Compute high order sub and error
	t1 = A.hi - B.hi;
	e = t1 - A.hi;
	//Compute low order term, including error and overflows
	t2 = ((-B.hi - e) + ( A.hi - (t1 - e))) + A.lo - B.lo;
	//Normalise to get final result
	C.hi = t1 + t2;
	C.lo = t2 -(C.hi - t1);

	return C;
}

//multiplication: Double * Double
__inline Double DoubleMul(Double A, Double B)
{
	Double C;
	float cona, conb, a1, a2, b1, b2;
	float c11, c21, c2, e, t1, t2;

	//Compute initial high order approximation and error
	//If a fused multiply-add is available
	//c11 = A.hi * B.hi;
	//c21 = A.hi * B.hi - c11;

	//If no fused multiply-add is available
	cona = A.hi * 8193.0f;
	conb = B.hi * 8193.0f;
	a1 = cona - (cona - A.hi);
	b1 = conb - (conb - B.hi);
	a2 = A.hi - a1;
	b2 = B.hi - b1;
	c11 = A.hi * B.hi;
	c21 = (((a1 * b1 - c11) + a1 * b2) + a2 * b1) + a2 * b2;

	//Compute high order word of mixed term:
	c2 = A.hi * B.lo + A.lo * B.hi;
	//Compute (c11, c21) + c2 using Knuth's trick, including low order product
	t1 = c11 + c2;
	e = t1 - c11;
	t2 = ((c2-e) + (c11 - (t1 -e))) + c21 + A.lo * B.lo;
	//Normalise to get final result
	C.hi = t1 + t2;
	C.lo = t2 - ( C.hi - t1);
	return C;
}

//divides: Double / Double
__inline Double DoubleDiv(Double A, Double B)
{
	Double C;
	float a1, a2, b1, b2, cona, conb, c11, c2, c21, e, s1, s2;
	float t1, t2, t11, t12, t21, t22;

	// Compute a DP approximation to the quotient.
	s2 = 1.0f / B.hi;
	s1 = A.hi * s2;

	//This splits s1 and b.x into high-order and low-order words.
	cona = s1 * 8193.0f;
	conb = B.hi * 8193.0f;
	a1 = cona - (cona - s1);
	b1 = conb - (conb - B.hi);
	a2 = s1 -a1;
	b2 = B.hi - b1;
	//Multiply s1 * dsb(1) using Dekker's method.
	c11 = s1 * B.hi;
	c21 = (((a1 * b1 - c11) + a1 * b2) + a2 * b1) + a2 * b2;
	//Compute s1 * b.lo (only high-order word is needed).
	c2 = s1 * B.lo;
	//Compute (c11, c21) + c2 using Knuth's trick.
	t1 = c11 + c2;
	e = t1 - c11;
	t2 = ((c2 - e) + (c11 - (t1 - e))) + c21;
	//The result is t1 + t2, after normalization.
	t12 = t1 + t2;
	t22 = t2 - (t12 - t1);
	//Compute dsa - (t12, t22) using Knuth's trick.
	t11 = A.hi - t12;
	e = t11 - A.hi;
	t21 = ((-t12 - e) + (A.hi - (t11 - e))) + A.lo - t22;
	//Compute high-order word of (t11, t21) and divide by b.hi.
	s2 *= (t11 + t21);
	//The result is s1 + s2, after normalization.
	C.hi = s1 + s2;
	C.lo = s2 - (C.hi - s1);

	return C;
}

#endif
