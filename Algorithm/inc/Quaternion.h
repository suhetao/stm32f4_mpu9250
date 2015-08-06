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

#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include "FastMath.h"

__inline void Quaternion_Add(float *r, float *a, float *b)
{
	r[0] = a[0] + b[0];
	r[1] = a[1] + b[1];
	r[2] = a[2] + b[2];
	r[3] = a[3] + b[3];
}

__inline void Quaternion_Sub(float *r, float *a, float *b)
{
	r[0] = a[0] - b[0];
	r[1] = a[1] - b[1];
	r[2] = a[2] - b[2];
	r[3] = a[3] - b[3];
}

__inline void Quaternion_Multiply(float *r, float *a, float *b)
{
	r[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
	r[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
	r[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
	r[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

__inline void Quaternion_Conjugate(float *r, float *a)
{
	r[0] = a[0];
	r[1] = -a[1];
	r[2] = -a[2];
	r[3] = -a[3];
}

__inline void Quaternion_Scalar(float *r, float *q, float scalar)
{
	r[0] = q[0] * scalar;
	r[1] = q[1] * scalar;
	r[2] = q[2] * scalar;
	r[3] = q[3] * scalar;
}

void Quaternion_Normalize(float *q);
void Quaternion_FromEuler(float *q, float *rpy);
void Quaternion_ToEuler(float *q, float* rpy);
void Quaternion_FromRotationMatrix(float *R, float *Q);
void Quaternion_RungeKutta4(float *q, float *w, float dt, int normalize);
void Quaternion_From6AxisData(float* q, float *accel, float *mag);

#endif
