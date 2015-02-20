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

#include "miniIMU.h"
#include "FastMath.h"
#include "Matrix.h"
//////////////////////////////////////////////////////////////////////////
//
//all parameters below need to be tune
#define EKF_PQ_INITIAL 0.001f
#define EKF_QQ_INITIAL 0.05f
#define EKF_RA_INITIAL 0.005346f

#if EKF_STATE_DIM == 7
#define EKF_PWB_INITIAL 0.001f
#define EKF_QWB_INITIAL 0.0000005f
#endif
//////////////////////////////////////////////////////////////////////////
//
static float I[EKF_STATE_DIM * EKF_STATE_DIM] = {
#if EKF_STATE_DIM == 4
	1.0f, 0, 0, 0,
	0, 1.0f, 0, 0,
	0, 0, 1.0f, 0,
	0, 0, 0, 1.0f,
#else //EKF_STATE_DIM == 7
	1.0f, 0, 0, 0, 0, 0, 0,
	0, 1.0f, 0, 0, 0, 0, 0,
	0, 0, 1.0f, 0, 0, 0, 0,
	0, 0, 0, 1.0f, 0, 0, 0,
	0, 0, 0, 0, 1.0f, 0, 0,
	0, 0, 0, 0, 0, 1.0f, 0,
	0, 0, 0, 0, 0, 0, 1.0f,
#endif
};

static float P[EKF_STATE_DIM * EKF_STATE_DIM] = {
#if EKF_STATE_DIM == 4
	EKF_PQ_INITIAL, 0, 0, 0,
	0, EKF_PQ_INITIAL, 0, 0,
	0, 0, EKF_PQ_INITIAL, 0,
	0, 0, 0, EKF_PQ_INITIAL,
#else //EKF_STATE_DIM == 7
	EKF_PQ_INITIAL, 0, 0, 0, 0, 0, 0,
	0, EKF_PQ_INITIAL, 0, 0, 0, 0, 0,
	0, 0, EKF_PQ_INITIAL, 0, 0, 0, 0,
	0, 0, 0, EKF_PQ_INITIAL, 0, 0, 0,
	0, 0, 0, 0, EKF_PWB_INITIAL, 0, 0,
	0, 0, 0, 0, 0, EKF_PWB_INITIAL, 0,
	0, 0, 0, 0, 0, 0, EKF_PWB_INITIAL,
#endif
};

static float Q[EKF_STATE_DIM * EKF_STATE_DIM] = {
#if EKF_STATE_DIM == 4
	EKF_QQ_INITIAL, 0, 0, 0,
	0, EKF_QQ_INITIAL, 0, 0,
	0, 0, EKF_QQ_INITIAL, 0,
	0, 0, 0, EKF_QQ_INITIAL,
#else //EKF_STATE_DIM == 7
	EKF_QQ_INITIAL, 0, 0, 0, 0, 0, 0,
	0, EKF_QQ_INITIAL, 0, 0, 0, 0, 0,
	0, 0, EKF_QQ_INITIAL, 0, 0, 0, 0,
	0, 0, 0, EKF_QQ_INITIAL, 0, 0, 0,
	0, 0, 0, 0, EKF_QWB_INITIAL, 0, 0,
	0, 0, 0, 0, 0, EKF_QWB_INITIAL, 0,
	0, 0, 0, 0, 0, 0, EKF_QWB_INITIAL,
#endif
};

static float RA[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM] = {
	EKF_RA_INITIAL, 0, 0,
	0, EKF_RA_INITIAL, 0,
	0, 0, EKF_RA_INITIAL,
};

static float F[EKF_STATE_DIM * EKF_STATE_DIM] = {
#if EKF_STATE_DIM == 4
	0.0f, 0, 0, 0,
	0, 0.0f, 0, 0,
	0, 0, 0.0f, 0,
	0, 0, 0, 0.0f,
#else //EKF_STATE_DIM == 7
	1.0f, 0, 0, 0, 0, 0, 0,
	0, 1.0f, 0, 0, 0, 0, 0,
	0, 0, 1.0f, 0, 0, 0, 0,
	0, 0, 0, 1.0f, 0, 0, 0,
	0, 0, 0, 0, 1.0f, 0, 0,
	0, 0, 0, 0, 0, 1.0f, 0,
	0, 0, 0, 0, 0, 0, 1.0f,
#endif
};

static float HA[EKF_MEASUREMENT_DIM * EKF_STATE_DIM] = {
#if EKF_STATE_DIM == 4
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
#else //EKF_STATE_DIM == 7
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0,
#endif
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

static float SA[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];

//static float Qd[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];
//static float Rd[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];
//static float AQ[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];

void EKF_IMUInit(float *accel, float *gyro)
{
	//NED coordinate system unit vector
	float nedVector[3] = {0, 0 , -1.0f};
	float accelVector[3] = {0, 0 , 0};
	float norm;
	float crossVector[3];
	float sinwi, cosw, sinhalfw, coshalfw;

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
	coshalfw = FastSqrt((1.0f + cosw) * 0.5f);
	sinhalfw = FastSqrt((1.0f - cosw) * 0.5f);

	X[0] = coshalfw;
	X[1] = crossVector[0] * sinhalfw;
	X[2] = crossVector[1] * sinhalfw;
	X[3] = crossVector[2] * sinhalfw;

	norm = FastSqrtI(X[0] * X[0] + X[1] * X[1] + X[2] * X[2] + X[3] * X[3]);
	X[0] *= norm;
	X[1] *= norm;
	X[2] *= norm;
	X[3] *= norm;
}

void EKF_IMUUpdate(float *gyro, float *accel, float dt)
{
	float norm;
	float halfdx, halfdy, halfdz;
	float neghalfdx, neghalfdy, neghalfdz;
#if EKF_STATE_DIM == 7
	float halfdtq0, neghalfdtq0, halfdtq1, neghalfdtq1,
		halfdtq2, neghalfdtq2, halfdtq3, neghalfdtq3;
#endif
	float halfdt = 0.5f * dt;
	//////////////////////////////////////////////////////////////////////////
	float _2q0,_2q1,_2q2,_2q3;
	float q0, q1, q2, q3;
	//
	float SB[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM] = {0};
	int SP[EKF_MEASUREMENT_DIM] = {0};
	float SI[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM] = {0};
	//////////////////////////////////////////////////////////////////////////
#if EKF_STATE_DIM == 4
	halfdx = halfdt * gyro[0];
	halfdy = halfdt * gyro[1];
	halfdz = halfdt * gyro[2];
#else //EKF_STATE_DIM == 7
	halfdx = halfdt * (gyro[0] - X[4]);
	halfdy = halfdt * (gyro[1] - X[5]);
	halfdz = halfdt * (gyro[2] - X[6]);
#endif
	neghalfdx = -halfdx;
	neghalfdy = -halfdy;
	neghalfdz = -halfdz;
	//
	q0 = X[0];
	q1 = X[1];
	q2 = X[2];
	q3 = X[3];
	
#if EKF_STATE_DIM == 4
	/* F[0] = 1.0f; */ F[1] = neghalfdx; F[2] = neghalfdy; F[3] = neghalfdz;
	F[4] = halfdx; /* F[5] = 1.0f; */ F[6] = neghalfdz;	F[7] = halfdy;
	F[8] = halfdy;	F[9] = halfdz;	/* F[10] = 1.0f; */ F[11] = neghalfdx;
	F[12] = halfdz; F[13] = neghalfdy; F[14] = halfdx; /* F[15] = 1.0f; */
#else
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
#endif

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
	Maxtrix_Mul(F, EKF_STATE_DIM, EKF_STATE_DIM, P, EKF_STATE_DIM, PX);
	Maxtrix_Mul_With_Transpose(PX, EKF_STATE_DIM, EKF_STATE_DIM, F, EKF_STATE_DIM, P);
	Maxtrix_Add(P, EKF_STATE_DIM, EKF_STATE_DIM, Q, P);

	//////////////////////////////////////////////////////////////////////////
	//measurement update
	//kalman gain calculation
	//K = P * H' / (R + H * P * H')
	_2q0 = 2.0f * X[0];
	_2q1 = 2.0f * X[1];
	_2q2 = 2.0f * X[2];
	_2q3 = 2.0f * X[3];

#if EKF_STATE_DIM == 4
	HA[0] = _2q2; HA[1] = -_2q3; HA[2] = _2q0; HA[3] = -_2q1;
	HA[4] = -_2q1; HA[5] = -_2q0; HA[6] = -_2q3; HA[7] = -_2q2;
	HA[8] = -_2q0; HA[9] = _2q1; HA[10] = _2q2; HA[11] = -_2q3;
#else //EKF_STATE_DIM == 7
	HA[0] = _2q2; HA[1] = -_2q3; HA[2] = _2q0; HA[3] = -_2q1;
	HA[8] = -_2q1; HA[9] = -_2q0; HA[10] = -_2q3; HA[11] = -_2q2;
	HA[14] = -_2q0; HA[15] = _2q1; HA[16] = _2q2; HA[17] = -_2q3;
#endif

	Maxtrix_Mul_With_Transpose(P, EKF_STATE_DIM, EKF_STATE_DIM, HA, EKF_MEASUREMENT_DIM, PXY);
	Maxtrix_Mul(HA, EKF_MEASUREMENT_DIM, EKF_STATE_DIM, PXY, EKF_MEASUREMENT_DIM, SA);
	Maxtrix_Add(SA, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM, RA, SA);
	//Matrix_Div(KA, PXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, SA, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM, Qd, Rd, AQ);
	Maxtrix_Inv(SA, SB, SP, EKF_MEASUREMENT_DIM, SI);
	Maxtrix_Mul(PXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, SI, EKF_MEASUREMENT_DIM, KA);

	//state measurement update
	//X = X + K * Y;
	Y[0] = -2.0f * (X[1] * X[3] - X[0] * X[2]);
	Y[1] = -2.0f * (X[2] * X[3] + X[0] * X[1]);
	Y[2] = 1.0f - 2.0f * (X[0] * X[0] + X[3] * X[3]);

	//normalize accel
	norm = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	accel[0] *= norm;
	accel[1] *= norm;
	accel[2] *= norm;

	Y[0] = accel[0] - Y[0];
	Y[1] = accel[1] - Y[1];
	Y[2] = accel[2] - Y[2];
	
	// Update State Vector
	Maxtrix_Mul(KA, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, Y, 1, KY);
	Maxtrix_Add(X, EKF_STATE_DIM, 1, KY, X);

	//normalize quaternion
	norm = FastSqrtI(X[0] * X[0] + X[1] * X[1] + X[2] * X[2] + X[3] * X[3]);
	X[0] *= norm;
	X[1] *= norm;
	X[2] *= norm;
	X[3] *= norm;

	//covariance measurement update
	//P = (I - K * H) * P
	//or
	//P=(I - K*H)*P*(I - K*H)' + K*R*K'
#if 1
	Maxtrix_Mul(KA, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, HA, EKF_STATE_DIM, PX);
	Maxtrix_Sub(I, EKF_STATE_DIM, EKF_STATE_DIM, PX, PX);
	Maxtrix_Mul(PX, EKF_STATE_DIM, EKF_STATE_DIM, P, EKF_STATE_DIM, PXX);
	Matrix_Copy(PXX, EKF_STATE_DIM, EKF_STATE_DIM, P);
#else
	Maxtrix_Mul(KA, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, HA, EKF_STATE_DIM, PX);
	Maxtrix_Sub(I, EKF_STATE_DIM, EKF_STATE_DIM, PX, PX);
	Maxtrix_Mul(PX, EKF_STATE_DIM, EKF_STATE_DIM, P, EKF_STATE_DIM, PXX);
	Maxtrix_Mul_With_Transpose(PXX, EKF_STATE_DIM, EKF_STATE_DIM, PX, EKF_STATE_DIM, P);
	Maxtrix_Mul(KA, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, RA, EKF_MEASUREMENT_DIM, PXY);
	Maxtrix_Mul_With_Transpose(PXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, KA, EKF_STATE_DIM, PX);
	Maxtrix_Add(P, EKF_STATE_DIM, EKF_STATE_DIM, PX, P);
#endif
}

void EKF_IMUGetAngle(float* rpy)
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
