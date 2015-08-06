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

#ifndef _STM32F4_NEMA_H
#define _STM32F4_NEMA_H

#include "stm32f4xx.h"

#include "Double.h"

#define NEMA_MESSAGEID_SIZE (5)
#define NMEA_MAXSVS         (12)

typedef enum{
	GPNON   = 0x0000,
	GPGGA   = 0x0001,
	GPGSA   = 0x0002,
	GPRMC   = 0x0004,
	GPVTG   = 0x0008
} NEMA_MessageType;

//GGA Global positioning system fix data
typedef struct _nmeaGPGGA
{
	double time;
	double lat;
	char NS;
	double lon;
	char EW;
	int quality;
	int numSV;
	double HDOP;
	double alt;
	char uAlt;
	double sep;
	char uSep;
	double diffAge;
	int diffStation;
}nmeaGPGGA, *PnmeaGPGGA;

//GSA GNSS DOP and Active Satellites
typedef struct _nmeaGPGSA
{
	char opMode;
	int navMode;
	int sv[NMEA_MAXSVS];//Satellite number
	double PDOP;
	double HDOP;
	double VDOP;
	int systemId;
} nmeaGPGSA, *PnmeaGPGSA;

//RMC Recommended Minimum data
typedef struct _nmeaGPRMC
{
	double time;
	char status;
	double lat;
	char NS;
	double lon;
	char EW;
	double spd;
	double cog;
	int date;
	double mv;
	char mvEW;
	char posMode;
	char navStatus;
} nmeaGPRMC, *PnmeaGPRMC;

//VTG Course over ground and Ground speed
typedef struct _nmeaGPVTG
{
	double cogt;
	char T;
	double cogm;
	char M;
	double knots;
	char N;
	double kph;
	char K;
	char posMode;
} nmeaGPVTG, *PnmeaGPVTG;

typedef struct _nmeaINFO
{
	double lat;
	double lon;
	double alt;
	double spd;
	double cog;
} nmeaINFO;

typedef uint64_t u64;

//////////////////////////////////////////////////////////////////////////
//input -2^23 < x < 2^23
__inline float NEMA_Fast_UintToFloat(u32 x)
{
	union { float f; u32 i; } u = { 8388608.0f };
	u.i |= x;
	return u.f - 8388608.0f;
}
//
__inline float NEMA_Fast_FloatInverse(float x)
{
	union { float f; u32 i; } u;
	u.f  = x;
	u.i = 0x7F000000 - u.i;
	return u.f;
}
//////////////////////////////////////////////////////////////////////////
//input -2^52 < x < 2^52
__inline double NEMA_Fast_Uint64ToDouble(u64 x)
{
	union { double d; u64 i; } u = {4503599627370496.0 };
	u.i |= x;
	return u.d - 4503599627370496.0;
}

__inline double NEMA_Fast_DoubleInverse(double x)
{
	union { double d; u64 i; } u;
	u.d = x;
	u.i = (u64)0x7FE0000000000000 - u.i;
	return u.d;
}

__inline u8 NEMA_FastCRCtoI(s8 *p, u8 *table){
	return table[*p] << 4 | table[*(p + 1)];
}

float NEMA_FastAtoF(s8 *p, s32 len);
double NEMA_FastAtoD(s8 *p, s32 len);
s32 NEMA_FastAtoI(s8 *p, s32 len);
u16 NEMA_Parser(s8 *p, u16 len);
s16 NEMA_GetMessage(void *data, s32* iType);

//
double NMEA_Convert2Degrees(double val);
double NMEA_Degree2Radian(double val);
double NMEA_Radian2Degree(double val);

Double NMEA_Degree2RadianD(Double val);

#endif
