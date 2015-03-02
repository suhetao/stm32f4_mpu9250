#include "FastMath.h"

typedef union {
	long i;
	float f;
}L2F;

float FastLn(float x)
{
	L2F e;	
	float xn;
	float	z;
	float	w;
	float	a;
	float	b;
	float	r;
	float	result;
	float znum, zden;
	
	int exponent = (*((int*)&x) & 0x7F800000) >> 23;
	e.i = (*((int*)&x) & 0x3F800000);

	if(e.f > ROOT_HALF){
		znum = e.f - 1.0f;
		zden = e.f * 0.5f + 0.5f;
	}
	else{
		exponent -= 1;
		znum = e.f - 0.5f;
		zden = e.f * 0.5f + 0.5f;
	}
	xn = (float)exponent;
	z = znum / zden;
	w = z * z;
	a = (LOGDA_COEF2 * w + LOGDA_COEF1) * w + LOGDA_COEF0;
	b = ((w + LOGDB_COEF2) * w + LOGDB_COEF1) * w + LOGDB_COEF0;
	r = a / b * w * z + z;
	result = xn * LN2_DC1 + r;
	r = xn * LN2_DC2;
	result += r;
	r = xn * LN2_DC3;
	result += r;
	return result;
}

float FastAsin(float x)
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

float FastAtan2(float y, float x)
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

float FastSqrtI(float x)
{
	L2F l2f;
	float xhalf = 0.5f * x;
	l2f.f = x;
	
	l2f.i = 0x5f3759df - (l2f.i >> 1);
	x = l2f.f * (1.5f - xhalf * l2f.f * l2f.f);
	return x;
}

float FastSqrt(float x)
{
	return x * FastSqrtI(x);
}
