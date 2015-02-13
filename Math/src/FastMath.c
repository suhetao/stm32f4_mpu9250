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

#define ROOT_HALF (0.70710678118654752440084436210485f)
#define LOGA_COEF0 (-4.649062303464e-1f)
#define LOGA_COEF1 (+1.360095468621e-2f)
#define LOGB_COEF0 (-5.578873750242f)

#define LOGDA_COEF0 (-6.4124943423745581147e1f)
#define LOGDA_COEF1 (+1.6383943563021534222e1f)
#define LOGDA_COEF2 (-7.8956112887491257267e-1f)
#define LOGDB_COEF0 (-7.6949932108494879777e2f)
#define LOGDB_COEF1 (+3.1203222091924532844e2f)
#define LOGDB_COEF2 (-3.5667977739034646171e1f)
#define LN2_DC1 (0.693359375f)
#define LN2_DC2 (-2.121944400e-4f)
#define LN2_DC3 (-5.46905827678e-14f)

__inline float32_t FastLn(float32_t x)
{
	Int2Float e;	
	float32_t xn;
	float32_t	z;
	float32_t	w;
	float32_t	a;
	float32_t	b;
	float32_t	r;
	float32_t	result;
	float32_t znum, zden;
	
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
	xn = (float32_t)exponent;
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

//translate from the DSP instruction of a DSP Library.
//#define PI (3.1415926535897932384626433832795f)
#define PI_2 (1.5707963267948966192313216916398f)
#define PI_3 (1.0471975511965977461542144610932f)
#define PI_4 (0.78539816339744830961566084581988f)
#define PI_6 (0.52359877559829887307710723054658f)
#define TWO_MINUS_ROOT3 (0.26794919243112270647255365849413f)
#define SQRT3_MINUS_1 (0.73205080756887729352744634150587f)
#define SQRT3 (1.7320508075688772935274463415059f)
#define EPS_FLOAT (+3.452669830012e-4f)
//Coefficients used for atan/atan2
#define ATANP_COEF0 (-1.44008344874F)
#define ATANP_COEF1 (-7.20026848898e-1F)
#define ATANQ_COEF0 (+4.32025038919F)
#define ATANQ_COEF1 (+4.75222584599F)
//Coefficients used for asin/acos
#define ASINP_COEF1 (-2.7516555290596F)
#define ASINP_COEF2 (+2.9058762374859F)
#define ASINP_COEF3 (-5.9450144193246e-1F)
#define ASINQ_COEF0 (-1.6509933202424e+1F)
#define ASINQ_COEF1 (+2.4864728969164e+1F)
#define ASINQ_COEF2 (-1.0333867072113e+1F)

__inline float32_t FastAsin(float32_t x)
{
    float32_t y, g;
    float32_t num, den, result;
    long i;
    float32_t sign = 1.0;
	
    y = x;
    if (y < (float32_t)0.0){
        y = -y;
        sign = -sign;
    }
		
    if (y > (float32_t)0.5){
        i = 1;
        if (y > (float32_t)1.0){
            result = 0.0;
            return result;
        }    
        g = (1.0f - y) * 0.5f;
        y = -2.0f * FastSqrt(g);
    }
    else{
        i = 0;
        if (y < (float32_t)EPS_FLOAT){
            result = y;
            if (sign < (float32_t)0.0){
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
    if (sign < (float32_t)0.0){
        result = -result;
    }
    return result;
}

__inline float32_t FastAtan2(float32_t y, float32_t x)
{
	float32_t f, g;
	float32_t num, den;
	float32_t result;
	int n;

	static const float32_t a[4] = {0, (float32_t)PI_6, (float32_t)PI_2, (float32_t)PI_3};

	if (x == (float32_t)0.0){
		if (y == (float32_t)0.0){
			result = 0.0;
			return result;
		}

		result = (float32_t)PI_2;
		if (y > (float32_t)0.0){
			return result;
		}
		if (y < (float32_t)0.0){
			result = -result;
			return result;
		}
	}
	n = 0;
	num = y;
	den = x;

	if (num < (float32_t)0.0){
		num = -num;
	}
	if (den < (float32_t)0.0){
		den = -den;
	}
	if (num > den){
		f = den;
		den = num;
		num = f;
		n = 2;
	}
	f = num / den;

	if (f > (float32_t)TWO_MINUS_ROOT3){
		num = f * (float)SQRT3_MINUS_1 - 1.0f + f;
		den = (float)SQRT3 + f;
		f = num / den;
		n = n + 1;
	}

	g = f;
	if (g < (float32_t)0.0){
		g = -g;
	}

	if (g < (float32_t)EPS_FLOAT){
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

	if (x < (float32_t)0.0){
		result = PI - result;
	}
	if (y < (float32_t)0.0){
		result = -result;
	}
	return result;
}
