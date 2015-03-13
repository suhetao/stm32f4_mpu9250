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

//translate from ADI's dsp library.
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

// Quake inverse square root
float FastSqrtI(float x)
{
#if 0
	L2F l2f;
	float xhalf = 0.5f * x;
	l2f.f = x;

	l2f.i = 0x5f3759df - (l2f.i >> 1);
	x = l2f.f * (1.5f - xhalf * l2f.f * l2f.f);
	return x;
#else
	/*
	unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
	float tmp = *(float*)&i;
	return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
	*/
	L2F l2f;
	l2f.f = x;
	l2f.i = 0x5F1F1412 - (l2f.i >> 1);
	return l2f.f * (1.69000231f - 0.714158168f * x * l2f.f * l2f.f);
#endif
}

float FastSqrt(float x)
{
	return x * FastSqrtI(x);
}

#define FAST_SIN_TABLE_SIZE 512

const float sinTable[FAST_SIN_TABLE_SIZE + 1] = {
	0.00000000f, 0.01227154f, 0.02454123f, 0.03680722f, 0.04906767f, 0.06132074f,
	0.07356456f, 0.08579731f, 0.09801714f, 0.11022221f, 0.12241068f, 0.13458071f,
	0.14673047f, 0.15885814f, 0.17096189f, 0.18303989f, 0.19509032f, 0.20711138f,
	0.21910124f, 0.23105811f, 0.24298018f, 0.25486566f, 0.26671276f, 0.27851969f,
	0.29028468f, 0.30200595f, 0.31368174f, 0.32531029f, 0.33688985f, 0.34841868f,
	0.35989504f, 0.37131719f, 0.38268343f, 0.39399204f, 0.40524131f, 0.41642956f,
	0.42755509f, 0.43861624f, 0.44961133f, 0.46053871f, 0.47139674f, 0.48218377f,
	0.49289819f, 0.50353838f, 0.51410274f, 0.52458968f, 0.53499762f, 0.54532499f,
	0.55557023f, 0.56573181f, 0.57580819f, 0.58579786f, 0.59569930f, 0.60551104f,
	0.61523159f, 0.62485949f, 0.63439328f, 0.64383154f, 0.65317284f, 0.66241578f,
	0.67155895f, 0.68060100f, 0.68954054f, 0.69837625f, 0.70710678f, 0.71573083f,
	0.72424708f, 0.73265427f, 0.74095113f, 0.74913639f, 0.75720885f, 0.76516727f,
	0.77301045f, 0.78073723f, 0.78834643f, 0.79583690f, 0.80320753f, 0.81045720f,
	0.81758481f, 0.82458930f, 0.83146961f, 0.83822471f, 0.84485357f, 0.85135519f,
	0.85772861f, 0.86397286f, 0.87008699f, 0.87607009f, 0.88192126f, 0.88763962f,
	0.89322430f, 0.89867447f, 0.90398929f, 0.90916798f, 0.91420976f, 0.91911385f,
	0.92387953f, 0.92850608f, 0.93299280f, 0.93733901f, 0.94154407f, 0.94560733f,
	0.94952818f, 0.95330604f, 0.95694034f, 0.96043052f, 0.96377607f, 0.96697647f,
	0.97003125f, 0.97293995f, 0.97570213f, 0.97831737f, 0.98078528f, 0.98310549f,
	0.98527764f, 0.98730142f, 0.98917651f, 0.99090264f, 0.99247953f, 0.99390697f,
	0.99518473f, 0.99631261f, 0.99729046f, 0.99811811f, 0.99879546f, 0.99932238f,
	0.99969882f, 0.99992470f, 1.00000000f, 0.99992470f, 0.99969882f, 0.99932238f,
	0.99879546f, 0.99811811f, 0.99729046f, 0.99631261f, 0.99518473f, 0.99390697f,
	0.99247953f, 0.99090264f, 0.98917651f, 0.98730142f, 0.98527764f, 0.98310549f,
	0.98078528f, 0.97831737f, 0.97570213f, 0.97293995f, 0.97003125f, 0.96697647f,
	0.96377607f, 0.96043052f, 0.95694034f, 0.95330604f, 0.94952818f, 0.94560733f,
	0.94154407f, 0.93733901f, 0.93299280f, 0.92850608f, 0.92387953f, 0.91911385f,
	0.91420976f, 0.90916798f, 0.90398929f, 0.89867447f, 0.89322430f, 0.88763962f,
	0.88192126f, 0.87607009f, 0.87008699f, 0.86397286f, 0.85772861f, 0.85135519f,
	0.84485357f, 0.83822471f, 0.83146961f, 0.82458930f, 0.81758481f, 0.81045720f,
	0.80320753f, 0.79583690f, 0.78834643f, 0.78073723f, 0.77301045f, 0.76516727f,
	0.75720885f, 0.74913639f, 0.74095113f, 0.73265427f, 0.72424708f, 0.71573083f,
	0.70710678f, 0.69837625f, 0.68954054f, 0.68060100f, 0.67155895f, 0.66241578f,
	0.65317284f, 0.64383154f, 0.63439328f, 0.62485949f, 0.61523159f, 0.60551104f,
	0.59569930f, 0.58579786f, 0.57580819f, 0.56573181f, 0.55557023f, 0.54532499f,
	0.53499762f, 0.52458968f, 0.51410274f, 0.50353838f, 0.49289819f, 0.48218377f,
	0.47139674f, 0.46053871f, 0.44961133f, 0.43861624f, 0.42755509f, 0.41642956f,
	0.40524131f, 0.39399204f, 0.38268343f, 0.37131719f, 0.35989504f, 0.34841868f,
	0.33688985f, 0.32531029f, 0.31368174f, 0.30200595f, 0.29028468f, 0.27851969f,
	0.26671276f, 0.25486566f, 0.24298018f, 0.23105811f, 0.21910124f, 0.20711138f,
	0.19509032f, 0.18303989f, 0.17096189f, 0.15885814f, 0.14673047f, 0.13458071f,
	0.12241068f, 0.11022221f, 0.09801714f, 0.08579731f, 0.07356456f, 0.06132074f,
	0.04906767f, 0.03680722f, 0.02454123f, 0.01227154f, 0.00000000f, -0.01227154f,
	-0.02454123f, -0.03680722f, -0.04906767f, -0.06132074f, -0.07356456f,
	-0.08579731f, -0.09801714f, -0.11022221f, -0.12241068f, -0.13458071f,
	-0.14673047f, -0.15885814f, -0.17096189f, -0.18303989f, -0.19509032f, 
	-0.20711138f, -0.21910124f, -0.23105811f, -0.24298018f, -0.25486566f, 
	-0.26671276f, -0.27851969f, -0.29028468f, -0.30200595f, -0.31368174f, 
	-0.32531029f, -0.33688985f, -0.34841868f, -0.35989504f, -0.37131719f, 
	-0.38268343f, -0.39399204f, -0.40524131f, -0.41642956f, -0.42755509f, 
	-0.43861624f, -0.44961133f, -0.46053871f, -0.47139674f, -0.48218377f, 
	-0.49289819f, -0.50353838f, -0.51410274f, -0.52458968f, -0.53499762f, 
	-0.54532499f, -0.55557023f, -0.56573181f, -0.57580819f, -0.58579786f, 
	-0.59569930f, -0.60551104f, -0.61523159f, -0.62485949f, -0.63439328f, 
	-0.64383154f, -0.65317284f, -0.66241578f, -0.67155895f, -0.68060100f, 
	-0.68954054f, -0.69837625f, -0.70710678f, -0.71573083f, -0.72424708f, 
	-0.73265427f, -0.74095113f, -0.74913639f, -0.75720885f, -0.76516727f, 
	-0.77301045f, -0.78073723f, -0.78834643f, -0.79583690f, -0.80320753f, 
	-0.81045720f, -0.81758481f, -0.82458930f, -0.83146961f, -0.83822471f, 
	-0.84485357f, -0.85135519f, -0.85772861f, -0.86397286f, -0.87008699f, 
	-0.87607009f, -0.88192126f, -0.88763962f, -0.89322430f, -0.89867447f, 
	-0.90398929f, -0.90916798f, -0.91420976f, -0.91911385f, -0.92387953f, 
	-0.92850608f, -0.93299280f, -0.93733901f, -0.94154407f, -0.94560733f, 
	-0.94952818f, -0.95330604f, -0.95694034f, -0.96043052f, -0.96377607f, 
	-0.96697647f, -0.97003125f, -0.97293995f, -0.97570213f, -0.97831737f, 
	-0.98078528f, -0.98310549f, -0.98527764f, -0.98730142f, -0.98917651f, 
	-0.99090264f, -0.99247953f, -0.99390697f, -0.99518473f, -0.99631261f, 
	-0.99729046f, -0.99811811f, -0.99879546f, -0.99932238f, -0.99969882f, 
	-0.99992470f, -1.00000000f, -0.99992470f, -0.99969882f, -0.99932238f, 
	-0.99879546f, -0.99811811f, -0.99729046f, -0.99631261f, -0.99518473f, 
	-0.99390697f, -0.99247953f, -0.99090264f, -0.98917651f, -0.98730142f, 
	-0.98527764f, -0.98310549f, -0.98078528f, -0.97831737f, -0.97570213f, 
	-0.97293995f, -0.97003125f, -0.96697647f, -0.96377607f, -0.96043052f, 
	-0.95694034f, -0.95330604f, -0.94952818f, -0.94560733f, -0.94154407f, 
	-0.93733901f, -0.93299280f, -0.92850608f, -0.92387953f, -0.91911385f, 
	-0.91420976f, -0.90916798f, -0.90398929f, -0.89867447f, -0.89322430f, 
	-0.88763962f, -0.88192126f, -0.87607009f, -0.87008699f, -0.86397286f, 
	-0.85772861f, -0.85135519f, -0.84485357f, -0.83822471f, -0.83146961f, 
	-0.82458930f, -0.81758481f, -0.81045720f, -0.80320753f, -0.79583690f, 
	-0.78834643f, -0.78073723f, -0.77301045f, -0.76516727f, -0.75720885f, 
	-0.74913639f, -0.74095113f, -0.73265427f, -0.72424708f, -0.71573083f, 
	-0.70710678f, -0.69837625f, -0.68954054f, -0.68060100f, -0.67155895f, 
	-0.66241578f, -0.65317284f, -0.64383154f, -0.63439328f, -0.62485949f, 
	-0.61523159f, -0.60551104f, -0.59569930f, -0.58579786f, -0.57580819f, 
	-0.56573181f, -0.55557023f, -0.54532499f, -0.53499762f, -0.52458968f, 
	-0.51410274f, -0.50353838f, -0.49289819f, -0.48218377f, -0.47139674f, 
	-0.46053871f, -0.44961133f, -0.43861624f, -0.42755509f, -0.41642956f, 
	-0.40524131f, -0.39399204f, -0.38268343f, -0.37131719f, -0.35989504f, 
	-0.34841868f, -0.33688985f, -0.32531029f, -0.31368174f, -0.30200595f, 
	-0.29028468f, -0.27851969f, -0.26671276f, -0.25486566f, -0.24298018f, 
	-0.23105811f, -0.21910124f, -0.20711138f, -0.19509032f, -0.18303989f, 
	-0.17096189f, -0.15885814f, -0.14673047f, -0.13458071f, -0.12241068f, 
	-0.11022221f, -0.09801714f, -0.08579731f, -0.07356456f, -0.06132074f, 
	-0.04906767f, -0.03680722f, -0.02454123f, -0.01227154f, -0.00000000f
};

void FastSinCos(float x, float *sinVal, float *cosVal)
{
	float fract, in; // Temporary variables for input, output
	unsigned short indexS, indexC; // Index variable
	float f1, f2, d1, d2; // Two nearest output values
	int n;
	float findex, Dn, Df, temp;

	// input x is in radians
	//Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi, for cosine add 0.25 (pi/2) to read sine table
	in = x * 0.159154943092f;

	// Calculation of floor value of input
	n = (int) in;

	// Make negative values towards -infinity
	if(in < 0.0f){
		n--;
	}
	// Map input value to [0 1]
	in = in - (float) n;

	// Calculation of index of the table
	findex = (float) FAST_SIN_TABLE_SIZE * in;
	indexS = ((unsigned short)findex) & 0x1ff;
	indexC = (indexS + (FAST_SIN_TABLE_SIZE / 4)) & 0x1ff;

	// fractional value calculation
	fract = findex - (float) indexS;

	// Read two nearest values of input value from the cos & sin tables
	f1 = sinTable[indexC+0];
	f2 = sinTable[indexC+1];
	d1 = -sinTable[indexS+0];
	d2 = -sinTable[indexS+1];

	Dn = 0.0122718463030f; // delta between the two points (fixed), in this case 2*pi/FAST_SIN_TABLE_SIZE
	Df = f2 - f1; // delta between the values of the functions
	temp = Dn*(d1 + d2) - 2*Df;
	temp = fract*temp + (3*Df - (d2 + 2*d1)*Dn);
	temp = fract*temp + d1*Dn;

	// Calculation of cosine value
	*cosVal = fract*temp + f1;

	// Read two nearest values of input value from the cos & sin tables
	f1 = sinTable[indexS+0];
	f2 = sinTable[indexS+1];
	d1 = sinTable[indexC+0];
	d2 = sinTable[indexC+1];

	Df = f2 - f1; // delta between the values of the functions
	temp = Dn*(d1 + d2) - 2*Df;
	temp = fract*temp + (3*Df - (d2 + 2*d1)*Dn);
	temp = fract*temp + d1*Dn;

	// Calculation of sine value
	*sinVal = fract*temp + f1;
}

float FastSin(float x)
{
	float sinVal, fract, in; // Temporary variables for input, output
	unsigned short index; // Index variable
	float a, b; // Two nearest output values
	int n;
	float findex;

	// input x is in radians
	// Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi
	in = x * 0.159154943092f;

	// Calculation of floor value of input
	n = (int) in;

	// Make negative values towards -infinity
	if(x < 0.0f){
		n--;
	}

	// Map input value to [0 1]
	in = in - (float) n;

	// Calculation of index of the table
	findex = (float) FAST_SIN_TABLE_SIZE * in;
	index = ((unsigned short)findex) & 0x1ff;

	// fractional value calculation
	fract = findex - (float) index;

	// Read two nearest values of input value from the sin table
	a = sinTable[index];
	b = sinTable[index+1];

	// Linear interpolation process
	sinVal = (1.0f-fract)*a + fract*b;

	// Return the output value
	return (sinVal);
}

float FastCos(float x)
{
	float cosVal, fract, in; // Temporary variables for input, output
	unsigned short index; // Index variable
	float a, b; // Two nearest output values
	int n;
	float findex;

	// input x is in radians
	// Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi, add 0.25 (pi/2) to read sine table
	in = x * 0.159154943092f + 0.25f;

	// Calculation of floor value of input
	n = (int) in;

	// Make negative values towards -infinity
	if(in < 0.0f){
		n--;
	}

	// Map input value to [0 1]
	in = in - (float) n;

	// Calculation of index of the table
	findex = (float) FAST_SIN_TABLE_SIZE * in;
	index = ((unsigned short)findex) & 0x1ff;

	// fractional value calculation
	fract = findex - (float) index;

	// Read two nearest values of input value from the cos table
	a = sinTable[index];
	b = sinTable[index+1];

	// Linear interpolation process
	cosVal = (1.0f-fract)*a + fract*b;

	// Return the output value
	return (cosVal);
}
