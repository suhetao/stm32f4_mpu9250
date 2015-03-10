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

#include "Quaternion.h"
#include "FastMath.h"

void Quaternion_Normalize(float *q)
{
	float norm = FastSqrtI(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= norm;
	q[1] *= norm;
	q[2] *= norm;
	q[3] *= norm;
}

void Quaternion_FromEuler(float *q, float *rpy)
{
	float sPhi2, cPhi2; // sin(phi/2) and cos(phi/2)
	float sThe2, cThe2; // sin(theta/2) and cos(theta/2)
	float sPsi2, cPsi2; // sin(psi/2) and cos(psi/2)
	// calculate sines and cosines
	
	FastSinCos(0.5f * rpy[0], &sPhi2, &cPhi2);
	FastSinCos(0.5f * rpy[1], &sThe2, &cThe2);
	FastSinCos(0.5f * rpy[2], &sPsi2, &cPsi2);
	
	// compute the quaternion elements
	q[0] = cPsi2 * cThe2 * cPhi2 + sPsi2 * sThe2 * sPhi2;
	q[1] = cPsi2 * cThe2 * sPhi2 - sPsi2 * sThe2 * cPhi2;
	q[2] = cPsi2 * sThe2 * cPhi2 + sPsi2 * cThe2 * sPhi2;
	q[3] = sPsi2 * cThe2 * cPhi2 - cPsi2 * sThe2 * sPhi2;
}

void Quaternion_FromRotationMatrix(float *R, float *Q)
{
#if 0
	// calculate the trace of the matrix
	float trace = R[0] + R[4] + R[8];
	float s;
	if(trace > 0){
		s = 0.5f * FastSqrt(trace + 1.0f);
		Q[0] = 0.25f / s;
		Q[1] = (R[7] - R[5]) * s;
		Q[2] = (R[2] - R[6]) * s;
		Q[3] = (R[3] - R[1]) * s;
	}
	else{
		if(R[0] > R[4] && R[0] > R[8] ){
			s = 0.5f * FastSqrtI(1.0f + R[0] - R[4] - R[8]);
			Q[0] = (R[7] - R[5]) * s;
			Q[1] = 0.25f / s;
			Q[2] = (R[1] + R[3]) * s;
			Q[3] = (R[2] + R[6]) * s;
		}
		else if(R[4] > R[8]) {
			s = 0.5f * FastSqrtI(1.0f + R[4] - R[0] - R[8]);
			Q[0] = (R[2] - R[6]) * s;
			Q[1] = (R[1] + R[3]) * s;
			Q[2] = 0.25f / s;
			Q[3] = (R[5] + R[7]) * s;
		}
		else{
			s = 0.5f * FastSqrtI(1.0f + R[8] - R[0] - R[4]);
			Q[0] = (R[3] - R[1]) * s;
			Q[1] = (R[2] + R[6]) * s;
			Q[2] = (R[5] + R[7]) * s;
			Q[3] = 0.25f / s;
		}
	}
#else
	// get the instantaneous orientation quaternion
	float fq0sq; // q0^2
	float recip4q0; // 1/4q0
	float fmag; // quaternion magnitude
#define SMALLQ0 0.01F // limit where rounding errors may appear
	// get q0^2 and q0
	fq0sq = 0.25f * (1.0f + R[0] + R[4] + R[8]);
	Q[0] = (float)FastSqrt(FastAbs(fq0sq));
	// normal case when q0 is not small meaning rotation angle not near 180 deg
	if (Q[0] > SMALLQ0){
		// calculate q1 to q3
		recip4q0 = 0.25F / Q[0];
		Q[1] = recip4q0 * (R[5] - R[7]);
		Q[2] = recip4q0 * (R[6] - R[2]);
		Q[3] = recip4q0 * (R[1] - R[3]);
	}
	else{
		// special case of near 180 deg corresponds to nearly symmetric matrix
		// which is not numerically well conditioned for division by small q0
		// instead get absolute values of q1 to q3 from leading diagonal
		Q[1] = FastSqrt(FastAbs(0.5f * (1.0f + R[0]) - fq0sq));
		Q[2] = FastSqrt(FastAbs(0.5f * (1.0f + R[4]) - fq0sq));
		Q[3] = FastSqrt(FastAbs(0.5f * (1.0f + R[8]) - fq0sq));
		// first assume q1 is positive and ensure q2 and q3 are consistent with q1
		if ((R[1] + R[3]) < 0.0f){
			// q1*q2 < 0 so q2 is negative
			Q[2] = -Q[2];
			if ((R[5] + R[7]) > 0.0f){
				// q1*q2 < 0 and q2*q3 > 0 so q3 is also both negative
				Q[3] = -Q[3];
			}
		}
		else if ((R[1] + R[3]) > 0.0f){
			if ((R[5] + R[7]) < 0.0f){
				// q1*q2 > 0 and q2*q3 < 0 so q3 is negative
				Q[3] = -Q[3];
			}
		}
		// negate the vector components if q1 should be negative
		if ((R[5] - R[7]) < 0.0f){
			Q[1] = -Q[1];
			Q[2] = -Q[2];
			Q[3] = -Q[3];
		}
	}
	// finally re-normalize
	fmag = FastSqrtI(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
	Q[0] *= fmag;
	Q[1] *= fmag;
	Q[2] *= fmag;
	Q[3] *= fmag;
#endif
}

void Quaternion_RungeKutta4(float *q, float *w, float dt, int normalize)
{
	float half = 0.5f;
	float two = 2.0f;
	float qw[4], k2[4], k3[4], k4[4];
	float tmpq[4], tmpk[4];

	//qw = q * w * half;
	Quaternion_Multiply(qw, q, w);
	Quaternion_Scalar(qw, qw, half);
	//k2 = (q + qw * dt * half) * w * half;
	Quaternion_Scalar(tmpk, qw, dt * half);
	Quaternion_Add(tmpk, q, tmpk);
	Quaternion_Multiply(k2, tmpk, w);
	Quaternion_Scalar(k2, k2, half);
	//k3 = (q + k2 * dt * half) * w * half;
	Quaternion_Scalar(tmpk, k2, dt * half);
	Quaternion_Add(tmpk, q, tmpk);
	Quaternion_Multiply(k3, tmpk, w);
	Quaternion_Scalar(k3, k3, half);
	//k4 = (q + k3 * dt) * w * half;
	Quaternion_Scalar(tmpk, k3, dt);
	Quaternion_Add(tmpk, q, tmpk);
	Quaternion_Multiply(k4, tmpk, w);
	Quaternion_Scalar(k4, k4, half);
	//q += (qw + k2 * two + k3 * two + k4) * (dt / 6);
	Quaternion_Scalar(tmpk, k2, two);
	Quaternion_Add(tmpq, qw, tmpk);
	Quaternion_Scalar(tmpk, k3, two);
	Quaternion_Add(tmpq, tmpq, tmpk);
	Quaternion_Add(tmpq, tmpq, k4);
	Quaternion_Scalar(tmpq, tmpq, dt / 6.0f);
	Quaternion_Add(q, q, tmpq);

	if (normalize){
		Quaternion_Normalize(q);
	}
}
