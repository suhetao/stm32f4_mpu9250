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
