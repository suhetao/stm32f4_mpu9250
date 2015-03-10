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

#include "Control.h"
#include "FastMath.h"

int Matrix_Inv3x3(float* A)
{
	// det = a11(a33a22-a32a23)-a21(a33a12-a32a13)+a31(a23a12-a22a13)
	// det = a00(a22a11-a21a12)-a10(a22a01-a21a02)+a20(a12a01-a11a02)
	float det;
	float M[9];
	// Invert the matrix
	/*
	| a11 a12 a13 |-1 | a33a22-a32a23 -(a33a12-a32a13) a23a12-a22a13 |
	| a21 a22 a23 | = 1/DET * | -(a33a21-a31a23) a33a11-a31a13 -(a23a11-a21a13) |
	| a31 a32 a33 | | a32a21-a31a22 -(a32a11-a31a12) a22a11-a21a12 |
	| a00 a01 a02 |-1 | a22a11-a21a12 -(a22a01-a21a02) a12a01-a11a02 |
	| a10 a11 a12 | = 1/DET * | -(a22a10-a20a12) a22a00-a20a02 -(a12a00-a10a02) |
	| a20 a21 a22 | | a21a10-a20a11 -(a21a00-a20a01) a11a00-a10a01 |
	*/
	det  = A[0] * (A[8] * A[4] - A[7] * A[5]) -
		A[3] * (A[8] * A[1] - A[7] * A[2]) +
		A[6] * (A[5] * A[1] - A[4] * A[2]);
	// Row 1
	// M[0] = (a22a11-a21a12)/det;
	M[0] = (A[8] * A[4] - A[7] * A[5]) / det;
	// M[1] = -(a22a01-a21a02)/det;
	M[1] = -(A[8] * A[1] - A[7] * A[2]) / det;
	// M[2] = (a12a01-a11a02)/det;
	M[2] = (A[5] * A[1] - A[4] * A[2]) / det;
	// Row 2
	// M[3] = -(a22a10-a20a12)/det;
	M[3] = -(A[8] * A[3] - A[6] * A[5]) / det;
	// M[4] = (a22a00-a20a02)/det;
	M[4] = (A[8] * A[0] - A[6] * A[2]) / det;
	// M[5] = -(a12a00-a10a02)/det;
	M[5] = -(A[5] * A[0] - A[3] * A[2]) / det;
	// Row 3
	// M[6] = (a21a10-a20a11)/det;
	M[6] = (A[7] * A[3] - A[6] * A[4]) / det;
	// M[7] = -(a21a00-a20a01)/det;
	M[7] = -(A[7] * A[0] - A[6] * A[1]) / det;
	// M[8] = (a11a00-a10a01)/det;
	M[8] = (A[4] * A[0] - A[3] * A[1]) / det;

	A[0] = M[0]; A[1] = M[1]; A[2] = M[2];
	A[3] = M[3]; A[4] = M[4]; A[5] = M[5];
	A[6] = M[6]; A[7] = M[7]; A[8] = M[8];
	
	return 1;
}

//sweep
void Matrix_Inv(float *A, int n)
{
	float d;
	int i, j, k;
	int kn, kk;
	int ln, lk;

	for (k = 0; k < n; ++k){
		kn = k * n;
		kk = kn + k;

		d = 1.0f / A[kk];
		A[kk] = d;

		for (i = 0; i < n; ++i){
			if (i != k){
				A[kn + i] *= -d;
			}
		}
		for (i = 0; i < n; ++i){
			if ( i != k){
				A[i * n + k] *= d;
			}
		}
		for (i = 0; i < n; ++i){
			if (i != k){
				ln = i * n;
				lk = ln + k;

				for (j = 0; j < n; ++j){
					if (j != k ){
						A[ln + j] += A[lk] * A[kn + j] / d; 
					}
				}
			}
		}
	}
}

static QuadrotorParameter gQuadrotorParameter = {
	0,
};

void EulerConv(float* dt, float *deta)
{
	float dx = dt[X];
	float dy = dt[Y];
	float dz = dt[Z];
	float psi = dt[PSI];
	float d;
	
	float spsi, cpsi;
	
	FastSinCos(psi, &spsi, &cpsi);
	
	d = FastSqrt(dx * dx + dy * dy + dz * dz);

	if (-0.001f < d && d < 0.001f){
		deta[ROLL] = 0;
	}
	else{
		deta[ROLL] = FastAsin((dx * spsi - dy * cpsi)/d);
	}
	
	deta[PITCH] = FastAtan2(dx * cpsi + dy * spsi, dz);

	if(deta[ROLL] < MIN_ANG){
		deta[ROLL] = MIN_ANG;
	}
	else if(deta[ROLL] > MAX_ANG){
		deta[ROLL] = MAX_ANG;
	}
	if(deta[PITCH] < MIN_ANG){
		deta[PITCH] = MIN_ANG; 
	}
	else if(deta[PITCH] > MAX_ANG){
		deta[PITCH] = MAX_ANG;
	}
	deta[YAW] = psi;
}

void TorqueConv(float *eta, float *deta, float *dt)
{
	float sphi, cphi;
	float stht, ctht;
	float spsi, cpsi;
	
	float C[9];
	float sum[9];
	float* I = gQuadrotorParameter.I;
	
	FastSinCos(eta[ROLL], &sphi, &cphi);
	FastSinCos(eta[PITCH], &stht, &ctht);
	FastSinCos(eta[YAW], &spsi, &cpsi);

	C[0] = 1; C[1] = 0; C[2] = -stht;
	C[3] = 0; C[4] = cphi; C[5] = sphi * ctht;
	C[6] = 0; C[7] = -sphi; C[8] = cphi * ctht;

	//torque = I * C * euler;
	sum[0] = I[0] * C[0] + I[1] * C[3] + I[2] * C[6];
	sum[1] = I[0] * C[1] + I[1] * C[4] + I[2] * C[7];
	sum[2] = I[0] * C[2] + I[1] * C[5] + I[2] * C[8];
	sum[3] = I[3] * C[0] + I[4] * C[3] + I[5] * C[6];
	sum[4] = I[3] * C[1] + I[4] * C[4] + I[5] * C[7];
	sum[5] = I[3] * C[2] + I[4] * C[5] + I[5] * C[8];
	sum[6] = I[6] * C[0] + I[7] * C[3] + I[8] * C[6];
	sum[7] = I[6] * C[1] + I[7] * C[4] + I[8] * C[7];
	sum[8] = I[6] * C[2] + I[7] * C[5] + I[8] * C[8];
	
	dt[0] = sum[0] * deta[ROLL] + sum[1] * deta[PITCH] + sum[2] * deta[YAW];
	dt[1] = sum[0] * deta[ROLL] + sum[1] * deta[PITCH] + sum[2] * deta[YAW];
	dt[2] = sum[0] * deta[ROLL] + sum[1] * deta[PITCH] + sum[2] * deta[YAW];
}

void ForceConv(float *eta, float dz, float *df)
{
	float sphi, cphi;
	float stht, ctht;
	float spsi, cpsi;
	
	float R[9];
	float m = gQuadrotorParameter.m;
	
	FastSinCos(eta[ROLL], &sphi, &cphi);
	FastSinCos(eta[PITCH], &stht, &ctht);
	FastSinCos(eta[YAW], &spsi, &cpsi);

	R[0] = cpsi * ctht; R[1] = cpsi * stht * sphi - spsi * cphi; R[2] = cpsi * stht * cphi + spsi * sphi;
	R[3] = spsi * ctht; R[4] = spsi * stht * sphi + cpsi * cphi; R[5] = spsi * stht * cphi - cpsi * sphi;	
	R[6] = -stht; R[7] = ctht * sphi; R[8] = ctht * cphi;
	
	Matrix_Inv3x3(R);
	//u = {0, 0, dz};
	//d = m * inv(R) * u;
	//fd = d[2];
	R[8] *= m;
	*df = R[8] * dz;
}

void TorqueInv(float *dt, float df, float *domega)
{
	float T[16];
	float u[4];
	float b = gQuadrotorParameter.b;
	float d = gQuadrotorParameter.d;
	float l = gQuadrotorParameter.l;

	T[0] = b; T[1] = b; T[2] = b; T[3] = b;
	T[4] = 0; T[5] = -l * b; T[6] = 0; T[7] = l * b;
	T[8] = -l * b; T[9] = 0; T[10] = l * b; T[11] = 0;
	T[12] = -d; T[13] = d; T[14] = -d; T[15] = d;

	u[0] = df; u[1] = dt[0]; u[2] = dt[1]; u[3] = dt[2];

	//omega = inv(T) * u;
	Matrix_Inv(T, 4);
	
	domega[0] = T[0] * u[0] + T[1] * u[1] + T[2] * u[2] + T[3] * u[3];
	domega[1] = T[4] * u[0] + T[5] * u[1] + T[6] * u[2] + T[7] * u[3];
	domega[2] = T[8] * u[0] + T[9] * u[1] + T[10] * u[2] + T[11] * u[3];
	domega[3] = T[12] * u[0] + T[13] * u[1] + T[14] * u[2] + T[15] * u[3];
}

// Newton-Euler model

void QuadrotorControl(float* task, float *q, float *u)
{
	task[0] = PID_Calculate(&gQuadrotorParameter.PostionX, gQuadrotorParameter.X - task[0]);
	task[1] = PID_Calculate(&gQuadrotorParameter.PostionY, gQuadrotorParameter.Y - task[1]);
	task[2] = PID_Calculate(&gQuadrotorParameter.PostionZ, gQuadrotorParameter.Z - task[2]);
	
	//EulerConv(
	// quadrotor linear acceleration along zE WRT E-frame
	// quadrotor angular acceleration around x2 WRT E-frame (roll)
	// theta quadrotor angular acceleration around y1 WRT E-frame (pitch)
	// quadrotor angular acceleration around zE WRT E-frame (yaw)
	
	// The first one shows how the quadrotor accelerates according to
	// the basic movement commands given.


	// The second system of equations explains how the basic movements are related
	// to the propellers’ squared speed.

	// The third equation takes into accounts the motors dynamics and shows the
	// relation between propellers’ speed and motors’ voltage.
}
