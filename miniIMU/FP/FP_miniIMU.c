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

#include "FP_miniIMU.h"
#include "FP_Math.h"
#include "FP_Matrix.h"
#include "FastMath.h"

//////////////////////////////////////////////////////////////////////////
//S16.16
//precision 1 / 2^16 = 0.0000152587890625

//all parameters below need to be tune
#define FP_EKF_PQ_INITIAL 66//0.001
#define FP_EKF_QQ_INITIAL 66//0.001
#define FP_EKF_RA_INITIAL 1638//0.025

#if FP_EKF_STATE_DIM == 7
#define FP_EKF_PWB_INITIAL 66//0.001f
#define FP_EKF_QWB_INITIAL 7//0.0001f
#endif
//////////////////////////////////////////////////////////////////////////
//
static Q16 I[FP_EKF_STATE_DIM * FP_EKF_STATE_DIM] = {
#if EKF_STATE_DIM == 4
	Q16_One, 0, 0, 0,
	0, Q16_One, 0, 0,
	0, 0, Q16_One, 0,
	0, 0, 0, Q16_One,
#else //EKF_STATE_DIM == 7
	Q16_One, 0, 0, 0, 0, 0, 0,
	0, Q16_One, 0, 0, 0, 0, 0,
	0, 0, Q16_One, 0, 0, 0, 0,
	0, 0, 0, Q16_One, 0, 0, 0,
	0, 0, 0, 0, Q16_One, 0, 0,
	0, 0, 0, 0, 0, Q16_One, 0,
	0, 0, 0, 0, 0, 0, Q16_One,
#endif
};

static Q16 P[FP_EKF_STATE_DIM * FP_EKF_STATE_DIM] = {
#if FP_EKF_STATE_DIM == 4
	FP_EKF_PQ_INITIAL, 0, 0, 0,
	0, FP_EKF_PQ_INITIAL, 0, 0,
	0, 0, FP_EKF_PQ_INITIAL, 0,
	0, 0, 0, FP_EKF_PQ_INITIAL,
#else //FP_EKF_STATE_DIM == 7
	FP_EKF_PQ_INITIAL, 0, 0, 0, 0, 0, 0,
	0, FP_EKF_PQ_INITIAL, 0, 0, 0, 0, 0,
	0, 0, FP_EKF_PQ_INITIAL, 0, 0, 0, 0,
	0, 0, 0, FP_EKF_PQ_INITIAL, 0, 0, 0,
	0, 0, 0, 0, FP_EKF_PWB_INITIAL, 0, 0,
	0, 0, 0, 0, 0, FP_EKF_PWB_INITIAL, 0,
	0, 0, 0, 0, 0, 0, FP_EKF_PWB_INITIAL,
#endif
};

static Q16 Q[FP_EKF_STATE_DIM * FP_EKF_STATE_DIM] = {
#if FP_EKF_STATE_DIM == 4
	FP_EKF_QQ_INITIAL, 0, 0, 0,
	0, FP_EKF_QQ_INITIAL, 0, 0,
	0, 0, FP_EKF_QQ_INITIAL, 0,
	0, 0, 0, FP_EKF_QQ_INITIAL,
#else //FP_EKF_STATE_DIM == 7
	FP_EKF_QQ_INITIAL, 0, 0, 0, 0, 0, 0,
	0, FP_EKF_QQ_INITIAL, 0, 0, 0, 0, 0,
	0, 0, FP_EKF_QQ_INITIAL, 0, 0, 0, 0,
	0, 0, 0, FP_EKF_QQ_INITIAL, 0, 0, 0,
	0, 0, 0, 0, FP_EKF_QWB_INITIAL, 0, 0,
	0, 0, 0, 0, 0, FP_EKF_QWB_INITIAL, 0,
	0, 0, 0, 0, 0, 0, FP_EKF_QWB_INITIAL,
#endif
};

static Q16 R[FP_EKF_MEASUREMENT_DIM * FP_EKF_MEASUREMENT_DIM] = {
	FP_EKF_RA_INITIAL, 0, 0,
	0, FP_EKF_RA_INITIAL, 0,
	0, 0, FP_EKF_RA_INITIAL,
};

static Q16 F[FP_EKF_STATE_DIM * FP_EKF_STATE_DIM] = {
#if FP_EKF_STATE_DIM == 4
	Q16_One, 0, 0, 0,
	0, Q16_One, 0, 0,
	0, 0, Q16_One, 0,
	0, 0, 0, Q16_One,
#else //FP_EKF_STATE_DIM == 7
	Q16_One, 0, 0, 0, 0, 0, 0,
	0, Q16_One, 0, 0, 0, 0, 0,
	0, 0, Q16_One, 0, 0, 0, 0,
	0, 0, 0, Q16_One, 0, 0, 0,
	0, 0, 0, 0, Q16_One, 0, 0,
	0, 0, 0, 0, 0, Q16_One, 0,
	0, 0, 0, 0, 0, 0, Q16_One,
#endif
};

static Q16 H[FP_EKF_MEASUREMENT_DIM * FP_EKF_STATE_DIM] = {
#if FP_EKF_STATE_DIM == 4
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
#else //FP_EKF_STATE_DIM == 7
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
#endif
};

//state
static Q16 X[FP_EKF_STATE_DIM];
static Q16 KY[FP_EKF_STATE_DIM];
//measurement
static Q16 Y[FP_EKF_MEASUREMENT_DIM];
//
static Q16 CBn[9];
//
static Q16 PX[FP_EKF_STATE_DIM * FP_EKF_STATE_DIM];
static Q16 PXX[FP_EKF_STATE_DIM * FP_EKF_STATE_DIM];
static Q16 PHT[FP_EKF_STATE_DIM * FP_EKF_MEASUREMENT_DIM];
static Q16 K[FP_EKF_STATE_DIM * FP_EKF_MEASUREMENT_DIM];
static Q16 S[FP_EKF_MEASUREMENT_DIM * FP_EKF_MEASUREMENT_DIM];

void FP_EKF_IMUInit(float *accel, float *gyro)
{
	//NED coordinate system unit vector
	float nedVector[3] = {0, 0 , -1.0f};
	float accelVector[3] = {0, 0 , 0};
	float norm;
	float crossVector[3];
	float sinwi, cosw, sinhalfw, coshalfw;
	float q[4];

	//unit accel
	norm = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	accelVector[0] = accel[0] * norm;
	accelVector[1] = accel[1] * norm;
	accelVector[2] = accel[2] * norm;

	//cross product between accel and reference
	crossVector[0] = accelVector[1] * nedVector[2] - accelVector[2] * nedVector[1];
	crossVector[1] = accelVector[2] * nedVector[0] - accelVector[0] * nedVector[2];
	crossVector[2] = accelVector[0] * nedVector[1] - accelVector[1] * nedVector[0];
	sinwi = FastSqrtI(crossVector[0] * crossVector[0] + crossVector[1] * crossVector[1] + crossVector[2] * crossVector[2]);
	crossVector[0] *= sinwi;
	crossVector[1] *= sinwi;
	crossVector[2] *= sinwi;

	//the angle between accel and reference is the dot product of the two vectors
	cosw = accelVector[0] * nedVector[0] + accelVector[1] * nedVector[1] + accelVector[2] * nedVector[2];
	coshalfw = FastSqrt(0.5f + 0.5f * cosw);
	sinhalfw = FastSqrt(0.5f - 0.5f * cosw);

	q[0] = coshalfw;
	q[1] = crossVector[0] * sinhalfw;
	q[2] = crossVector[1] * sinhalfw;
	q[3] = crossVector[2] * sinhalfw;

	X[0] = FT_Q16(q[0]);
	X[1] = FT_Q16(q[1]);
	X[2] = FT_Q16(q[2]);
	X[3] = FT_Q16(q[3]);
}

void FP_EKF_IMUUpdate(float *gyro, float *accel, float dt)
{
	Q16 halfdx, halfdy, halfdz;
	Q16 neghalfdx, neghalfdy, neghalfdz;
#if FP_EKF_STATE_DIM == 7
	Q16 halfdtq0, neghalfdtq0, halfdtq1, neghalfdtq1,
		halfdtq2, neghalfdtq2, halfdtq3, neghalfdtq3;
#endif
	Q16 halfdt = FP_SMUL(32768, FT_Q16(dt));
	//////////////////////////////////////////////////////////////////////////
	Q16 _2q0,_2q1,_2q2,_2q3;
	Q16 q0, q1, q2, q3;
	
	Q16 SI[FP_EKF_MEASUREMENT_DIM * FP_EKF_MEASUREMENT_DIM] = {0};
	//////////////////////////////////////////////////////////////////////////
	int __al, __ah;
	Q16 gx = FT_Q16(gyro[0]);
	Q16 gy = FT_Q16(gyro[1]);
	Q16 gz = FT_Q16(gyro[2]);
	//
	//////////////////////////////////////////////////////////////////////////
	float norm;
	Q16 qNorm;

#if FP_EKF_STATE_DIM == 4
	halfdx = FP_SMUL(halfdt, gx);
	halfdy = FP_SMUL(halfdt, gy);
	halfdz = FP_SMUL(halfdt, gz);
#else //EKF_STATE_DIM == 7
	halfdx = FP_SMUL(halfdt, FP_SUBS(gx, X[4]));
	halfdy = FP_SMUL(halfdt, FP_SUBS(gy, X[5]));
	halfdz = FP_SMUL(halfdt, FP_SUBS(gz, X[6]));
#endif
	neghalfdx = -halfdx;
	neghalfdy = -halfdy;
	neghalfdz = -halfdz;
	//
	q0 = X[0];
	q1 = X[1];
	q2 = X[2];
	q3 = X[3];

#if FP_EKF_STATE_DIM == 4
	/* F[0] = Q16_One; */ F[1] = neghalfdx; F[2] = neghalfdy; F[3] = neghalfdz;
	F[4] = halfdx; /* F[5] = Q16_One; */ F[6] = neghalfdz;	F[7] = halfdy;
	F[8] = halfdy;	F[9] = halfdz;	/* F[10] = Q16_One; */ F[11] = neghalfdx;
	F[12] = halfdz; F[13] = neghalfdy; F[14] = halfdx; /* F[15] = Q16_One; */
#else
	halfdtq0 = FP_SMUL(halfdt, q0);
	neghalfdtq0 = -halfdtq0;
	halfdtq1 = FP_SMUL(halfdt, q1);
	neghalfdtq1 = -halfdtq1;
	halfdtq2 = FP_SMUL(halfdt, q2);
	neghalfdtq2 = -halfdtq2;
	halfdtq3 = FP_SMUL(halfdt, q3);
	neghalfdtq3 = -halfdtq3;
	
	/* F[0] = Q16_One; */ F[1] = neghalfdx; F[2] = neghalfdy; F[3] = neghalfdz;
	F[4] = halfdtq1; F[5] = halfdtq2; F[6] = halfdtq3;
	F[7] = halfdx; /* F[8] = Q16_One; */ F[9] = halfdz;	F[10] = neghalfdy;
	F[11] = neghalfdtq0; F[12] = halfdtq3; F[13] = neghalfdtq2;
	F[14] = halfdy;	F[15] = neghalfdz;	/* F[16] = Q16_One; */ F[17] = halfdx;
	F[18] = neghalfdtq3; F[19] = neghalfdtq0; F[20] = halfdtq1;
	F[21] = halfdz; F[22] = halfdy; F[23] = neghalfdx; /* F[24] = Q16_One; */
	F[25] = halfdtq2; F[26] = neghalfdtq1; F[27] = neghalfdtq0;
#endif

	//////////////////////////////////////////////////////////////////////////
	//Extended Kalman Filter: Prediction Step
	//state time propagation
	//Update Quaternion with the new gyroscope measurements
	//X[0] = q0 - halfdx * q1 - halfdy * q2 - halfdz * q3;
	//X[1] = q1 + halfdx * q0 - halfdy * q3 + halfdz * q2;
	//X[2] = q2 + halfdx * q3 + halfdy * q0 - halfdz * q1;
	//X[3] = q3 - halfdx * q2 + halfdy * q1 + halfdz * q0;
	//cortex-m3's instruction assembly optimization
	__asm{
		smull __al, __ah, neghalfdx, q1;
		smlal __al, __ah, neghalfdy, q2;
		smlal __al, __ah, neghalfdz, q3;
		lsls __ah, __ah, #16;
		orr X[0], __ah, __al, lsr #16;
		adds X[0], q0, X[0];

		smull __al, __ah, halfdx, q0;
		smlal __al, __ah, neghalfdy, q3;
		smlal __al, __ah, halfdz, q2;
		lsls __ah, __ah, #16;
		orr X[1], __ah, __al, lsr #16;
		adds X[1], q1, X[1];

		smull __al, __ah, halfdx, q3;
		smlal __al, __ah, halfdy, q0;
		smlal __al, __ah, neghalfdz, q1;
		lsls __ah, __ah, #16;
		orr X[2], __ah, __al, lsr #16;
		adds X[2], q2, X[2];

		smull __al, __ah, neghalfdx, q2;
		smlal __al, __ah, halfdy, q1;
		smlal __al, __ah, halfdz, q0;
		lsls __ah, __ah, #16;
		orr X[3], __ah, __al, lsr #16;
		adds X[3], q3, X[3];
	}
	
	//covariance time propagation
	//P = F*P*F' + Q;
	FP_Matrix_Multiply(F, FP_EKF_STATE_DIM, FP_EKF_STATE_DIM, P, FP_EKF_STATE_DIM, PX);
	FP_Matrix_Multiply_With_Transpose(PX, FP_EKF_STATE_DIM, FP_EKF_STATE_DIM, F, FP_EKF_STATE_DIM, P);
	FP_Maxtrix_Add(P, FP_EKF_STATE_DIM, FP_EKF_STATE_DIM, Q, P);
	
	//////////////////////////////////////////////////////////////////////////
	//measurement update
	//kalman gain calculation
	//K = P * H' / (R + H * P * H')
	_2q0 = X[0] << 1;
	_2q1 = X[1] << 1;
	_2q2 = X[2] << 1;
	_2q3 = X[3] << 1;

#if FP_EKF_STATE_DIM == 4
	H[0] = _2q2; H[1] = -_2q3; H[2] = _2q0; H[3] = -_2q1;
	H[4] = -_2q1; H[5] = -_2q0; H[6] = -_2q3; H[7] = -_2q2;
	H[8] = -_2q0; H[9] = _2q1; H[10] = _2q2; H[11] = -_2q3;
#else //FP_EKF_STATE_DIM == 7
	H[0] = _2q2; H[1] = -_2q3; H[2] = _2q0; H[3] = -_2q1;
	H[8] = -_2q1; H[9] = -_2q0; H[10] = -_2q3; H[11] = -_2q2;
	H[14] = -_2q0; H[15] = _2q1; H[16] = _2q2; H[17] = -_2q3;
#endif

	FP_Matrix_Multiply_With_Transpose(P, FP_EKF_STATE_DIM, FP_EKF_STATE_DIM, H, FP_EKF_MEASUREMENT_DIM, PHT);
	FP_Matrix_Multiply(H, FP_EKF_MEASUREMENT_DIM, FP_EKF_STATE_DIM, PHT, FP_EKF_MEASUREMENT_DIM, S);
	FP_Maxtrix_Add(S, FP_EKF_MEASUREMENT_DIM, FP_EKF_MEASUREMENT_DIM, R, S);
	FP_Matrix_Inverse(S, FP_EKF_MEASUREMENT_DIM, SI);
	FP_Matrix_Multiply(PHT, FP_EKF_STATE_DIM, FP_EKF_MEASUREMENT_DIM, SI, FP_EKF_MEASUREMENT_DIM, K);

	//state measurement update
	//X = X + K * Y;
	//Y[0] = -2.0f * (X[1] * X[3] - X[0] * X[2]);
	//Y[1] = -2.0f * (X[2] * X[3] + X[0] * X[1]);
	//Y[2] = 1.0f - 2.0f * (X[0] * X[0] + X[3] * X[3]);
	Y[0] = FP_SUBS(FP_SMUL(X[0], X[2]), FP_SMUL(X[1], X[3])) << 1;
	Y[1] = FP_ADDS(FP_SMUL(-X[2], X[3]), FP_SMUL(-X[0], X[1])) << 1;
	Y[2] = FP_SUBS(Q16_One, FP_ADDS(FP_SMUL(X[0], X[0]), FP_SMUL(X[3], X[3])) << 1);

	//normalize accel
	norm = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	accel[0] *= norm;
	accel[1] *= norm;
	accel[2] *= norm;

	Y[0] = FP_SUBS(FT_Q16(accel[0]), Y[0]);
	Y[1] = FP_SUBS(FT_Q16(accel[1]), Y[1]);
	Y[2] = FP_SUBS(FT_Q16(accel[2]), Y[2]);
	FP_Matrix_Multiply(K, FP_EKF_STATE_DIM, FP_EKF_MEASUREMENT_DIM, Y, 1, KY);
	FP_Maxtrix_Add(X, FP_EKF_STATE_DIM, 1, X, KY);

	//normalize quaternion
	//norm = FastSqrtI(X[0] * X[0] + X[1] * X[1] + X[2] * X[2] + X[3] * X[3]);
	//X[0] *= norm;
	//X[1] *= norm;
	//X[2] *= norm;
	//X[3] *= norm;
	//cortex-m3's instruction assembly optimization
	__asm{
		smull __al, __ah, X[0], X[0];
		smlal __al, __ah, X[1], X[1];
		smlal __al, __ah, X[2], X[2];
		smlal __al, __ah, X[3], X[3];
		lsls __ah, __ah, #16;
		orr qNorm, __ah, __al, lsr #16;
	}
	qNorm = FP_SqrtI(qNorm, PRECISION);
	X[0] = FP_SMUL(X[0], qNorm);
	X[1] = FP_SMUL(X[1], qNorm);
	X[2] = FP_SMUL(X[2], qNorm);
	X[3] = FP_SMUL(X[3], qNorm);

	//covariance measurement update
	//P = (I - K * H) * P
	FP_Matrix_Multiply(K, FP_EKF_STATE_DIM, FP_EKF_MEASUREMENT_DIM, H, FP_EKF_STATE_DIM, PX);
	FP_Maxtrix_Sub(I, FP_EKF_STATE_DIM, FP_EKF_STATE_DIM, PX, PX);
	FP_Matrix_Multiply(PX, FP_EKF_STATE_DIM, FP_EKF_STATE_DIM, P, FP_EKF_STATE_DIM, PXX);
	FP_Matrix_Copy(PXX, FP_EKF_STATE_DIM, FP_EKF_STATE_DIM, P);
}

void FP_EKF_IMUGetAngle(float* rpy)
{
	Q16 q0q0 = FP_SMUL(X[0], X[0]);
	Q16 fpRPY[3];

	CBn[0] = FP_SUBS(((q0q0 + FP_SMUL(X[1], X[1])) << 1), Q16_One);
	CBn[1] = (FP_SMUL(X[1], X[2]) + FP_SMUL(X[0], X[3])) << 1;
	CBn[2] = (FP_SMUL(X[1], X[3]) - FP_SMUL(X[0], X[2])) << 1;
	//CBn[3] = 2.0f * (X[1] * X[2] - X[0] * X[3]);
	//CBn[4] = 2.0f * (q0q0 + X[2] * X[2]) - 1.0f;
	CBn[5] = (FP_SMUL(X[2], X[3]) + FP_SMUL(X[0], X[1])) << 1;
	//CBn[6] = 2.0f * (X[1] * X[3] + X[0] * X[2]);
	//CBn[7] = 2.0f * (X[2] * X[3] - X[0] * X[1]);
	CBn[8] = FP_SUBS(((q0q0 + FP_SMUL(X[3], X[3])) << 1), Q16_One);

	//roll
	fpRPY[0] = FP_FastAtan2(CBn[5], CBn[8]);
	if(fpRPY[0] == Q16_PI){
		fpRPY[0] = -Q16_PI;
	}
	//pitch
	if (CBn[2] >= Q16_One){
		fpRPY[1] = -Q16_HALFPI;
	}
	else if (fpRPY[2] <= -Q16_One){
		fpRPY[1] = Q16_HALFPI;
	}
	else{
		fpRPY[1] = FP_FastAsin(-CBn[2]);
	}
	//yaw
	fpRPY[2] = FP_FastAtan2(CBn[1], CBn[0]);
	if (fpRPY[2] < 0){
		fpRPY[2] = FP_ADDS(fpRPY[2], Q16_TWOPI);
	}
	if (fpRPY[2] >= Q16_TWOPI){
		fpRPY[2] = 0;
	}

	rpy[0] = TO_FLOAT_DEGREE(fpRPY[0]);
	rpy[1] = TO_FLOAT_DEGREE(fpRPY[1]);
	rpy[2] = TO_FLOAT_DEGREE(fpRPY[2]);
}
