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

#include "FastMath.h"
#include "Quaternion.h"
#include "miniAHRS.h"
#include "miniMatrix.h"
//////////////////////////////////////////////////////////////////////////
//
//all parameters below need to be tune
#define EKF_PQ_INITIAL 0.001f
#define EKF_PWB_INITIAL 0.001f

#define EKF_QQ_INITIAL 0.05f
#define EKF_QWB_INITIAL 0.0000005f

#define EKF_RA_INITIAL 0.005346f
#define EKF_RM_INITIAL 0.005346f
//////////////////////////////////////////////////////////////////////////
//
#ifdef UPDATE_P_COMPLICATED
static float I[EKF_STATE_DIM * EKF_STATE_DIM] = {
	1.0f, 0, 0, 0, 0, 0, 0,
	0, 1.0f, 0, 0, 0, 0, 0,
	0, 0, 1.0f, 0, 0, 0, 0,
	0, 0, 0, 1.0f, 0, 0, 0,
	0, 0, 0, 0, 1.0f, 0, 0,
	0, 0, 0, 0, 0, 1.0f, 0,
	0, 0, 0, 0, 0, 0, 1.0f,
};
#endif

static float P[EKF_STATE_DIM * EKF_STATE_DIM] = {
	EKF_PQ_INITIAL, 0, 0, 0, 0, 0, 0,
	0, EKF_PQ_INITIAL, 0, 0, 0, 0, 0,
	0, 0, EKF_PQ_INITIAL, 0, 0, 0, 0,
	0, 0, 0, EKF_PQ_INITIAL, 0, 0, 0,
	0, 0, 0, 0, EKF_PWB_INITIAL, 0, 0,
	0, 0, 0, 0, 0, EKF_PWB_INITIAL, 0,
	0, 0, 0, 0, 0, 0, EKF_PWB_INITIAL,
};

static float Q[EKF_STATE_DIM * EKF_STATE_DIM] = {
	EKF_QQ_INITIAL, 0, 0, 0, 0, 0, 0,
	0, EKF_QQ_INITIAL, 0, 0, 0, 0, 0,
	0, 0, EKF_QQ_INITIAL, 0, 0, 0, 0,
	0, 0, 0, EKF_QQ_INITIAL, 0, 0, 0,
	0, 0, 0, 0, EKF_QWB_INITIAL, 0, 0,
	0, 0, 0, 0, 0, EKF_QWB_INITIAL, 0,
	0, 0, 0, 0, 0, 0, EKF_QWB_INITIAL,
};

static float RA[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM] = {
	EKF_RA_INITIAL, 0, 0,
	0, EKF_RA_INITIAL, 0,
	0, 0, EKF_RA_INITIAL,
};

static float RM[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM] = {
	EKF_RM_INITIAL, 0, 0,
	0, EKF_RM_INITIAL, 0,
	0, 0, EKF_RM_INITIAL,
};

static float F[EKF_STATE_DIM * EKF_STATE_DIM] = {
	1.0f, 0, 0, 0, 0, 0, 0,
	0, 1.0f, 0, 0, 0, 0, 0,
	0, 0, 1.0f, 0, 0, 0, 0,
	0, 0, 0, 1.0f, 0, 0, 0,
	0, 0, 0, 0, 1.0f, 0, 0,
	0, 0, 0, 0, 0, 1.0f, 0,
	0, 0, 0, 0, 0, 0, 1.0f,
};

static float HA[EKF_MEASUREMENT_DIM * EKF_STATE_DIM] = {
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
};

static float HM[EKF_MEASUREMENT_DIM * EKF_STATE_DIM] = {
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
};

//state
static float X[EKF_STATE_DIM];
static float KY[EKF_STATE_DIM];
//measurement
static float Y[EKF_MEASUREMENT_DIM];
//
static float CBn[9];
//
static float PX[EKF_STATE_DIM * EKF_STATE_DIM];
static float PXX[EKF_STATE_DIM * EKF_STATE_DIM];
static float PXY[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
static float KA[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
static float KM[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
static float S[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];

void EKF_AHRSInit(float *accel, float *mag)
{
	float norm, normaccel;
	float sinroll, cosroll, sinpitch, cospitch;
	float HX, HY;
	float RPY[3];

	//normalize accel
	normaccel = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	accel[0] *= normaccel;
	accel[1] *= normaccel;
	accel[2] *= normaccel;

	norm = FastSqrtI(accel[1] * accel[1] + accel[2] * accel[2]);
	sinroll = accel[1] * norm;
	cosroll = FastSqrt(1.0f - sinroll * sinroll);
	sinpitch = -accel[0] * normaccel;
	cospitch = FastSqrt(1.0f - sinpitch * sinpitch);

	HX = mag[0] * cospitch + mag[2] * sinpitch;
	HY = mag[0] * sinroll * sinpitch + mag[1] * cosroll - mag[2] * sinroll * cospitch;

	// calculate the roll and pitch
	RPY[0] = FastAtan2(sinroll, cosroll);
	RPY[1] = FastAtan2(sinpitch, cospitch);
	// calculate the heading angle
	RPY[2] = -FastAtan2(HY, HX);

	Quaternion_FromEuler(X, RPY);
}

void EKF_AHRSUpdate(float *gyro, float *accel, float *mag, float dt)
{
	float norm;
	float halfdx, halfdy, halfdz;
	float neghalfdx, neghalfdy, neghalfdz;

	float halfdtq0, neghalfdtq0, halfdtq1, neghalfdtq1,
		halfdtq2, neghalfdtq2, halfdtq3, neghalfdtq3;
	float halfdt = 0.5f * dt;
	//////////////////////////////////////////////////////////////////////////
	float _2q0,_2q1,_2q2,_2q3;
	float q0, q1, q2, q3;
	//
	float SI[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM] = {0};
	//////////////////////////////////////////////////////////////////////////
	halfdx = halfdt * (gyro[0] - X[4]);
	halfdy = halfdt * (gyro[1] - X[5]);
	halfdz = halfdt * (gyro[2] - X[6]);
	neghalfdx = -halfdx;
	neghalfdy = -halfdy;
	neghalfdz = -halfdz;
	//
	q0 = X[0];
	q1 = X[1];
	q2 = X[2];
	q3 = X[3];

	halfdtq0 = halfdt * q0;
	neghalfdtq0 = -halfdtq0;
	halfdtq1 = halfdt * q1;
	neghalfdtq1 = -halfdtq1;
	halfdtq2 = halfdt * q2;
	neghalfdtq2 = -halfdtq2;
	halfdtq3 = halfdt * q3;
	neghalfdtq3 = -halfdtq3;

	/* F[0] = 1.0f; */ F[1] = neghalfdx; F[2] = neghalfdy; F[3] = neghalfdz;
	F[4] = halfdtq1; F[5] = halfdtq2; F[6] = halfdtq3;
	F[7] = halfdx; /* F[8] = 1.0f; */ F[9] = halfdz;	F[10] = neghalfdy;
	F[11] = neghalfdtq0; F[12] = halfdtq3; F[13] = neghalfdtq2;
	F[14] = halfdy;	F[15] = neghalfdz;	/* F[16] = 1.0f; */ F[17] = halfdx;
	F[18] = neghalfdtq3; F[19] = neghalfdtq0; F[20] = halfdtq1;
	F[21] = halfdz; F[22] = halfdy; F[23] = neghalfdx; /* F[24] = 1.0f; */
	F[25] = halfdtq2; F[26] = neghalfdtq1; F[27] = neghalfdtq0;

	//////////////////////////////////////////////////////////////////////////
	//Extended Kalman Filter: Prediction Step
	//state time propagation
	//Update Quaternion with the new gyroscope measurements
	X[0] = q0 - halfdx * q1 - halfdy * q2 - halfdz * q3;
	X[1] = q1 + halfdx * q0 - halfdy * q3 + halfdz * q2;
	X[2] = q2 + halfdx * q3 + halfdy * q0 - halfdz * q1;
	X[3] = q3 - halfdx * q2 + halfdy * q1 + halfdz * q0;

	//covariance time propagation
	//P = F*P*F' + Q;
	Matrix_Multiply(F, EKF_STATE_DIM, EKF_STATE_DIM, P, EKF_STATE_DIM, PX);
	Matrix_Multiply_With_Transpose(PX, EKF_STATE_DIM, EKF_STATE_DIM, F, EKF_STATE_DIM, P);
	Maxtrix_Add(P, EKF_STATE_DIM, EKF_STATE_DIM, Q, P);

	//////////////////////////////////////////////////////////////////////////
	//measurement update
	//kalman gain calculation
	//K = P * H' / (R + H * P * H')
	//acceleration of gravity
	Matrix_Multiply_With_Transpose(P, EKF_STATE_DIM, EKF_STATE_DIM, HA, EKF_MEASUREMENT_DIM, PXY);
	Matrix_Multiply(HA, EKF_MEASUREMENT_DIM, EKF_STATE_DIM, PXY, EKF_MEASUREMENT_DIM, S);
	Maxtrix_Add(S, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM, RA, S);
	Matrix_Inverse(S, EKF_MEASUREMENT_DIM, SI);
	Matrix_Multiply(PXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, SI, EKF_MEASUREMENT_DIM, KA);

	//state measurement update
	//X = X + K * Y;
	Y[0] = -2.0f * (X[1] * X[3] - X[0] * X[2]);
	Y[1] = -2.0f * (X[2] * X[3] + X[0] * X[1]);
	Y[2] = X[0] * X[0] - X[1] * X[1] - X[2] * X[2] + X[3] * X[3];;
	
	//calculate the relative jacobian matrix
	_2q0 = 2.0f * X[0];
	_2q1 = 2.0f * X[1];
	_2q2 = 2.0f * X[2];
	_2q3 = 2.0f * X[3];
	
	HA[0] = _2q2; HA[1] = -_2q3; HA[2] = _2q0; HA[3] = -_2q1;
	HA[8] = -_2q1; HA[9] = -_2q0; HA[10] = -_2q3; HA[11] = -_2q2;
	HA[14] = -_2q0; HA[15] = _2q1; HA[16] = _2q2; HA[17] = -_2q3;

	//normalize accel
	norm = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	accel[0] *= norm;
	accel[1] *= norm;
	accel[2] *= norm;

	Y[0] = accel[0] - Y[0];
	Y[1] = accel[1] - Y[1];
	Y[2] = accel[2] - Y[2];

	// Update State Vector
	Matrix_Multiply(KA, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, Y, 1, KY);
	Maxtrix_Add(X, EKF_STATE_DIM, 1, KY, X);
	
	//covariance measurement update
	//P = (I - K * H) * P
	//P = P - K * H * P
	//or
	//P=(I - K*H)*P*(I - K*H)' + K*R*K'
#ifndef UPDATE_P_COMPLICATED
	Matrix_Multiply(KA, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, HA, EKF_STATE_DIM, PX);
	Matrix_Multiply(PX, EKF_STATE_DIM, EKF_STATE_DIM, P, EKF_STATE_DIM, PXX);
	Maxtrix_Add(P, EKF_STATE_DIM, EKF_STATE_DIM, PXX, P);
#else
	Matrix_Multiply(KA, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, HA, EKF_STATE_DIM, PX);
	Maxtrix_Sub(I, EKF_STATE_DIM, EKF_STATE_DIM, PX, PX);
	Matrix_Multiply(PX, EKF_STATE_DIM, EKF_STATE_DIM, P, EKF_STATE_DIM, PXX);
	Matrix_Multiply_With_Transpose(PXX, EKF_STATE_DIM, EKF_STATE_DIM, PX, EKF_STATE_DIM, P);
	Matrix_Multiply(KA, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, RA, EKF_MEASUREMENT_DIM, PXY);
	Matrix_Multiply_With_Transpose(PXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, KA, EKF_STATE_DIM, PX);
	Maxtrix_Add(P, EKF_STATE_DIM, EKF_STATE_DIM, PX, P);
#endif

	//////////////////////////////////////////////////////////////////////////
	//measurement update
	//kalman gain calculation
	//K = P * H' / (R + H * P * H')
	//magetic
	Matrix_Multiply_With_Transpose(P, EKF_STATE_DIM, EKF_STATE_DIM, HM, EKF_MEASUREMENT_DIM, PXY);
	Matrix_Multiply(HM, EKF_MEASUREMENT_DIM, EKF_STATE_DIM, PXY, EKF_MEASUREMENT_DIM, S);
	Maxtrix_Add(S, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM, RM, S);
	Matrix_Inverse(S, EKF_MEASUREMENT_DIM, SI);
	Matrix_Multiply(PXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, SI, EKF_MEASUREMENT_DIM, KM);

	//state measurement update
	//X = X + K * Y;
	Y[0] = X[0] * X[0] + X[1] * X[1] - X[2] * X[2] - X[3] * X[3];
	Y[1] = 2.0f * (X[1] * X[2] - X[3] * X[0]);
	Y[2] = 2.0f * (X[1] * X[3] + X[2] * X[0]);
	
	//calculate the relative jacobian matrix
	_2q0 = 2.0f * X[0];
	_2q1 = 2.0f * X[1];
	_2q2 = 2.0f * X[2];
	_2q3 = 2.0f * X[3];
	HM[0] = _2q0; HM[1] = _2q1;	HM[2] = -_2q2; HM[3] = -_2q3;
	HM[4] = -_2q3; HM[5] = _2q2; HM[6] = _2q1; HM[7] = -_2q0;
	HM[8] = _2q2; HM[9] = _2q3; HM[10] = _2q0; HM[11] = _2q1;

	//normalize accel
	norm = FastSqrtI(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	mag[0] *= norm;
	mag[1] *= norm;
	mag[2] *= norm;

	Y[0] = mag[0] - Y[0];
	Y[1] = mag[1] - Y[1];
	Y[2] = mag[2] - Y[2];

	// Update State Vector
	Matrix_Multiply(KM, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, Y, 1, KY);
	Maxtrix_Add(X, EKF_STATE_DIM, 1, KY, X);

	//normalize quaternion
	norm = FastSqrtI(X[0] * X[0] + X[1] * X[1] + X[2] * X[2] + X[3] * X[3]);
	X[0] *= norm;
	X[1] *= norm;
	X[2] *= norm;
	X[3] *= norm;

	//covariance measurement update
	//P = (I - K * H) * P
	//P = P - K * H * P
	//or
	//P=(I - K*H)*P*(I - K*H)' + K*R*K'
#ifndef UPDATE_P_COMPLICATED
	Matrix_Multiply(KM, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, HM, EKF_STATE_DIM, PX);
	Matrix_Multiply(PX, EKF_STATE_DIM, EKF_STATE_DIM, P, EKF_STATE_DIM, PXX);
	Maxtrix_Add(P, EKF_STATE_DIM, EKF_STATE_DIM, PXX, P);
#else
	Matrix_Multiply(KM, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, HM, EKF_STATE_DIM, PX);
	Maxtrix_Sub(I, EKF_STATE_DIM, EKF_STATE_DIM, PX, PX);
	Matrix_Multiply(PX, EKF_STATE_DIM, EKF_STATE_DIM, P, EKF_STATE_DIM, PXX);
	Matrix_Multiply_With_Transpose(PXX, EKF_STATE_DIM, EKF_STATE_DIM, PX, EKF_STATE_DIM, P);
	Matrix_Multiply(KM, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, RM, EKF_MEASUREMENT_DIM, PXY);
	Matrix_Multiply_With_Transpose(PXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, KM, EKF_STATE_DIM, PX);
	Maxtrix_Add(P, EKF_STATE_DIM, EKF_STATE_DIM, PX, P);
#endif

}

void EKF_AHRSGetAngle(float* rpy)
{
	float q0q0 = X[0] * X[0];

	//x-y-z ned
	CBn[0] = 2.0f * (q0q0 + X[1] * X[1]) - 1.0f;
	CBn[1] = 2.0f * (X[1] * X[2] + X[0] * X[3]);
	CBn[2] = 2.0f * (X[1] * X[3] - X[0] * X[2]);
	//CBn[3] = 2.0f * (X[1] * X[2] - X[0] * X[3]);
	//CBn[4] = 2.0f * (q0q0 + X[2] * X[2]) - 1.0f;
	CBn[5] = 2.0f * (X[2] * X[3] + X[0] * X[1]);
	//CBn[6] = 2.0f * (X[1] * X[3] + X[0] * X[2]);
	//CBn[7] = 2.0f * (X[2] * X[3] - X[0] * X[1]);
	CBn[8] = 2.0f * (q0q0 + X[3] * X[3]) - 1.0f;

	//roll
	rpy[0] = FastAtan2(CBn[5], CBn[8]);
	if (rpy[0] == EKF_PI)
		rpy[0] = -EKF_PI;
	//pitch
	if (CBn[2] >= 1.0f)
		rpy[1] = -EKF_HALFPI;
	else if (CBn[2] <= -1.0f)
		rpy[1] = EKF_HALFPI;
	else
		rpy[1] = FastAsin(-CBn[2]);
	//yaw
	rpy[2] = FastAtan2(CBn[1], CBn[0]);
	if (rpy[2] < 0.0f){
		rpy[2] += EKF_TWOPI;
	}
	if (rpy[2] >= EKF_TWOPI){
		rpy[2] = 0.0f;
	}

	rpy[0] = EKF_TODEG(rpy[0]);
	rpy[1] = EKF_TODEG(rpy[1]);
	rpy[2] = EKF_TODEG(rpy[2]);
}
