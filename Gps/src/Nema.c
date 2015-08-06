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

#include "Memory.h"
#include "Nema.h"
#include "Queue.h"

//gobal queue variable
Queue mQueue;
Queue *pQueue = &mQueue;
//
static u8 HexTable[] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x01,
	0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 
	0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0A, 0x0B, 0x0C,
	0x0D, 0x0E, 0x0F
};

static const float ReciprocalFloat[] = {
	1.0f, 0.1f, 0.01f, 0.001f, 0.0001f,
	0.00001f, 0.000001f, 0.0000001f, 0.00000001f,
	0.000000001f, 0.0000000001f, 0.00000000001f, 0.000000000001f
};

//////////////////////////////////////////////////////////////////////////
float NEMA_FastAtoF(s8 *p, s32 len)
{
	u32 integral, fractional, nReciprocal, sign = 0;
	s32 i, hasfractional = 0;
	union { float f; u32 i; } fReciprocal;
	
	//////////////////////////////////////////////////////////////////////////
	//
	if (*p++ == '-') {
		sign = 1;
	}
	//
	//////////////////////////////////////////////////////////////////////////
	for (integral = 0, i = 0; *p != '.' && i++ < len; p++) {
		integral = integral * 10 + (*p - '0');
	}
	// Get digits after decimal point, if any.
	if (*p++ == '.') {
		i++;
		hasfractional = 1;
		for (fractional = 0; i++ < len; p++) {
			fractional = fractional * 10 + (*p - '0');
			nReciprocal++;
		}
	}
	if (!hasfractional){
		return NEMA_Fast_UintToFloat(integral);
	}
	else{
		fReciprocal.f = ReciprocalFloat[nReciprocal];
		if(!sign){
			//return NEMA_Fast_UintToFloat(integral) + NEMA_Fast_UintToFloat(fractional) * NEMA_Fast_FloatInverse(NEMA_Fast_UintToFloat(decimal));
			return NEMA_Fast_UintToFloat(integral) + NEMA_Fast_UintToFloat(fractional) * fReciprocal.f;
		}
		//fReciprocal.f = NEMA_Fast_UintToFloat(integral) + NEMA_Fast_UintToFloat(fractional) * NEMA_Fast_FloatInverse(NEMA_Fast_UintToFloat(decimal));
		fReciprocal.f = NEMA_Fast_UintToFloat(integral) + NEMA_Fast_UintToFloat(fractional) * fReciprocal.f;
		fReciprocal.i |= 0x80000000;
		return fReciprocal.f;
	}
}

static const double ReciprocalDouble[] = {
	1.0, 0.1, 0.01, 0.001, 0.0001,
	0.00001, 0.000001, 0.0000001, 0.00000001,
	0.000000001, 0.0000000001, 0.00000000001, 0.000000000001
};

double NEMA_FastAtoD(s8 *p, s32 len)
{
	u32 integral, fractional, nReciprocal, sign = 0;
	s32 i, hasfractional = 0;
	union { double d; u64 i; } dReciprocal;

	//////////////////////////////////////////////////////////////////////////
	//
	if (*p++ == '-') {
		sign = 1;
	}
	//
	for (integral = 0, i = 0; *p != '.' && i++ < len; p++) {
		integral = integral * 10 + (*p - '0');
	}
	// Get digits after decimal point, if any.
	if (*p++ == '.') {
		i++;
		hasfractional = 1;
		for (fractional = 0; i++ < len; p++) {
			fractional = fractional * 10 + (*p - '0');
			nReciprocal++;
		}
	}
	if (!hasfractional){
		return NEMA_Fast_Uint64ToDouble(integral);
	}
	else{
		dReciprocal.d = ReciprocalDouble[nReciprocal];
		if(!sign){
			//return NEMA_Fast_Uint64ToDouble(integral) + NEMA_Fast_Uint64ToDouble(fractional) * NEMA_Fast_DoubleInverse(NEMA_Fast_Uint64ToDouble(decimal));
			return NEMA_Fast_Uint64ToDouble(integral) + NEMA_Fast_Uint64ToDouble(fractional) * dReciprocal.d;
		}
		//dReciprocal.d NEMA_Fast_Uint64ToDouble(integral) + NEMA_Fast_Uint64ToDouble(fractional) * NEMA_Fast_DoubleInverse(NEMA_Fast_Uint64ToDouble(decimal));
		dReciprocal.d = NEMA_Fast_Uint64ToDouble(integral) + NEMA_Fast_Uint64ToDouble(fractional) * dReciprocal.d;
		dReciprocal.i |= (u64)0x8000000000000000;
		return dReciprocal.d;
	}
}

s32 NEMA_FastAtoI(s8 *p, s32 len)
{
	s32 i, integral;
	u32 sign = 0;
	//////////////////////////////////////////////////////////////////////////
	//
	if (*p++ == '-') {
		sign = 1;
	}
	//
	//////////////////////////////////////////////////////////////////////////
	for (integral = 0, i = 0;i++ < len; p++) {
		integral = integral * 10 + (*p - '0');
	}
	if(!sign){
		return -integral;
	}
	return integral;
}

static const s8 *NEMA_MessageID[] = {
	//Number of fields except Message ID, Checksum, Carriage return and line feed
	"GGA", //17 - 3 = 14
	"GSA", //21 - 3 = 18
	//"GSV", //8..16 - 3 = 5..13
	"RMC", //16 - 3 = 13
	"VTG", //12 - 3 = 9
};

s16 NEMA_GetMessage(void *data, s32* iType)
{
	Buff buff;
	s32 ret;
	u16 queueSize = Queue_Size(pQueue);
	u16 useQueueSize = 0, numFields = 0;
	s16 find = 0, SVs = 0;
	//
	NEMA_MessageType messageType = GPNON;
	*iType = GPNON;

	if(!queueSize){
		return -1;
	}
	for(; useQueueSize < queueSize; useQueueSize++){
		ret = Queue_Dequeue(pQueue, &buff);
		if(ret == 0){
			//////////////////////////////////////////////////////////////////////////
			//find messageID
			if(find == 0){
				if(buff.Len != NEMA_MESSAGEID_SIZE){
					continue;
				}
				else if(0 == MemCmp((u8*)(buff.Buff + 2), (u8*)NEMA_MessageID[0], 3)){
					messageType = GPGGA;
					numFields = 14;
				}
				else if(0 == MemCmp((u8*)(buff.Buff + 2), (u8*)NEMA_MessageID[1], 3)){
					messageType = GPGSA;
					numFields = 18;
				}
				else if(0 == MemCmp((u8*)(buff.Buff + 2), (u8*)NEMA_MessageID[2], 3)){
					messageType = GPRMC;
					numFields = 13;
				}
				else if(0 == MemCmp((u8*)(buff.Buff + 2), (u8*)NEMA_MessageID[3], 3)){
					messageType = GPVTG;
					numFields = 9;
				}
				find = 1;
				break;
			}
		}
	}
	if(find && (numFields <= queueSize - useQueueSize)){
		switch(numFields){
			case 14: //GGA
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPGGA)data)->time = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPGGA)data)->lat = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: North/South indicator always 'N' or 'S'
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPGGA)data)->lon = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: East/West indicator always 'E' or 'W'
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPGGA)data)->quality = NEMA_FastAtoI(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPGGA)data)->numSV = NEMA_FastAtoI(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPGGA)data)->HDOP = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPGGA)data)->alt = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: Altitude units: meters (fixed field) always 'M'
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPGGA)data)->sep = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: Separation units: meters always 'M'
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: Age of differential corrections (blank when DGPS is not used)
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: ID of station providing differential corrections (blank when DGPS is not used)
				break;
			case 18: //GSA
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: Operation mode always 'A'
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPGSA)data)->navMode = NEMA_FastAtoI(buff.Buff, buff.Len);
				for(;SVs < NMEA_MAXSVS; SVs++){
					ret = Queue_Dequeue(pQueue, &buff);
					((PnmeaGPGSA)data)->sv[SVs] = NEMA_FastAtoI(buff.Buff, buff.Len);
				}
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPGSA)data)->PDOP = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPGSA)data)->HDOP = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPGSA)data)->VDOP = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: GNSS System ID
				break;
			case 13: //RMC
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPRMC)data)->time = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: status always 'A'
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPRMC)data)->lat = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: North/South indicator always 'N' or 'S'
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPRMC)data)->lon = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: East/West indicator always 'E' or 'W'
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPRMC)data)->spd = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPRMC)data)->cog = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPRMC)data)->date = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: Magnetic variation value (blank - not supported)
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: Magnetic variation E/W indicator (blank - notsupported)
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: Mode Indicator
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: Navigational status indicator 'V'
				break;
			case 9: //VTG
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPVTG)data)->cogt = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: true always 'T'
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPVTG)data)->cogm = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: magnetic always 'M'
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPVTG)data)->knots = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: knots always 'N'
				ret = Queue_Dequeue(pQueue, &buff);
				((PnmeaGPVTG)data)->kph = NEMA_FastAtoD(buff.Buff, buff.Len);
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: kilometers per hour always 'K'
				ret = Queue_Dequeue(pQueue, &buff);
				//skip Fixed field: Mode Indicator always 'A'
				break;
		}
		*iType = messageType;
	}
	return find;
}

u16 NEMA_Parser(s8 *p, u16 len)
{
	s8 *dataStart = p;
	s8 *dataEnd = p + len;
	u16 size, useSize = 0;
	u16 startPos, startCRCPos, endCRCPos;
	s8 null = 0;
	s8 crc, calCRC = 0;
	//////////////////////////////////////////////////////////////////////////
	u16 findStart = 0, findEnd = 0;
	s32 error = 0;
	//////////////////////////////////////////////////////////////////////////
	//
	for(startPos = useSize;((dataStart < dataEnd) && (!findEnd) && (error == 0)); dataStart++, useSize++){
		switch(*dataStart)
		{
			//find start code '$'
		case '$':
			startPos = useSize + 1;
			findStart = 1;
			startCRCPos = startPos;
			break;
			//find fields
		case '*':
			endCRCPos = useSize;
		case ',':
			size = useSize - startPos;
			//A null field 
			if (!size){
				error = Queue_Enqueue(pQueue, &null, 1);
			}
			else{
				error = Queue_Enqueue(pQueue, p + startPos, size);
			}
			startPos = useSize + 1;
			break;
			//find tail
			//'*' + [CRC] + '\r' + '\n'
		case '\n':
			findEnd = 1;
			crc = NEMA_FastCRCtoI(p + startPos, HexTable);
			break;
		}
	}
	if(findStart && findEnd){
		for (;startCRCPos < endCRCPos; startCRCPos++){
			calCRC ^= p[startCRCPos];
		}
		if (crc != calCRC){
			//why will crc be different?
			//that means something bad happened in serial communication 
		}
	}
	if(error < 0){
		//why will queue be fully?
		//error handler
	}
	//return len - useSize;
	return useSize;
}

double NMEA_Convert2Degrees(double val)
{
	Double onehundred = {100.0f, 0.0f};
	Double sixty = {60.0f, 0.0f};
	
	// double degree = ((int)(val / 100));
	Double dval = doubleToDouble(val);
	Double ddegree = DoubleDiv(dval, onehundred);
	double degree = (s32)DoubleTodouble(ddegree);
	
	//val = degree + (val - degree * 100) / 60;
	ddegree = doubleToDouble(degree);
	dval = DoubleAdd(ddegree, DoubleDiv(DoubleSub(dval, DoubleMul(ddegree, onehundred)), sixty));
	
	return DoubleTodouble(dval);
}

double NMEA_Degree2Radian(double val)
{
	Double dret = DoubleMul(doubleToDouble(val),
		doubleToDouble(0.01745329251994329576923690768489));
	
	return DoubleTodouble(dret);
}

Double NMEA_Degree2RadianD(Double val)
{
	return DoubleMul(val, doubleToDouble(0.01745329251994329576923690768489));
}

double NMEA_Radian2Degree(double val)
{
	Double dret = DoubleMul(doubleToDouble(val),
		doubleToDouble(57.295779513082320876798154814105));
	
	return DoubleTodouble(dret);
}
