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

#include "Map.h"
#include "Nema.h"

#include "FastMath.h"

DoublePoint Map_BLToGauss(Double latitude, Double longitude)
{
	DoublePoint point;
	
	int ProjNo = 0, ZoneWide = 6;
	__int64 X0, Y0;

	Double lon, lat, _lon;
	Double e2, ee, NN, T, C, A, M;
	//
	float sin, cos;
	float tan, _2sin, _4sin, _6sin;
	Double dsin, dcos;
	Double dtan, _d2sin, _d4sin, _d6sin;
	//beijing 54
	Double a = doubleToDouble(6378245.0);
	Double f = doubleToDouble(0.00335232986925913509889373114314);
	
	//xi'an 80
	//Double a = doubleToDouble(6378140.0);
	//Double f = doubleToDouble(0.00335281317789691440603238146967);

	ProjNo = (int)DoubleTodouble(DoubleDiv(lon, intToDouble(ZoneWide)));
	_lon = intToDouble(ProjNo * ZoneWide + ZoneWide / 2);
	_lon = NMEA_Degree2RadianD(_lon);
	lon = NMEA_Degree2RadianD(longitude);
	lat = NMEA_Degree2RadianD(latitude);
	
	tan = (float)DoubleTodouble(lat);
	dtan = floatToDouble(FastTan(tan));
	FastSinCos((float)DoubleTodouble(lat), &sin, &cos);
	dsin = floatToDouble(sin);
	dcos = floatToDouble(cos);
	
	_2sin = (float)DoubleTodouble(DoubleMul(intToDouble(2), lat));
	_4sin = (float)DoubleTodouble(DoubleMul(intToDouble(4), lat));
	_6sin = (float)DoubleTodouble(DoubleMul(intToDouble(6), lat));
	_d2sin = floatToDouble(FastSin(_2sin));
	_d4sin = floatToDouble(FastSin(_4sin));
	_d6sin = floatToDouble(FastSin(_6sin));
	
	e2 = DoubleSub(DoubleMul(intToDouble(2),f), DoubleMul(f,f));
	ee = DoubleMul(e2, DoubleSub(intToDouble(1), e2));
	NN = DoubleMul(a, FastSqrtID(DoubleSub(doubleToDouble(1.0), DoubleMul(e2, DoubleMul(dsin, dsin)))));
	T = DoubleMul(dtan, dtan);
	C = DoubleMul(ee, DoubleMul(dcos, dcos));
	A = DoubleMul(DoubleSub(lon, _lon), dcos);

	//todo not finish yet!
/*
	M = a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * lat
		- (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) * _d2sin
		+ (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * _d4sin
		- (35 * e2 * e2 * e2 / 3072) * _d6sin);
	point.lon = NN * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ee) * A * A * A * A * A / 120);
	point.lat = M + NN * tan * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 + (61 - 58 * T + T * T + 600 * C - 330 * ee) * A * A * A * A * A * A / 720);
*/
	X0 = 1000000L * (ProjNo + 1) + 500000L;
	Y0 = 0;
	point.lon = DoubleAdd(point.lon, doubleToDouble(NEMA_Fast_Uint64ToDouble(X0)));
	point.lat = DoubleAdd(point.lat, doubleToDouble(NEMA_Fast_Uint64ToDouble(Y0)));

	return point;
}

//////////////////////////////////////////////////////////////////////////
//must init with local two reference points including lat, lon and the length
//betweeb two reference points 

void Map_Init(Map* pMap, double _lat, double _lon, double lat, double lon,
	double x, double y, double length)
{
	Double dic;
	
	pMap->X = doubleToDouble(x);
	pMap->Y = doubleToDouble(y);
	
	pMap->_dPoint.lat = doubleToDouble(_lat); pMap->_dPoint.lon = doubleToDouble(_lon);
	pMap->dPoint.lat = doubleToDouble(lat); pMap->_dPoint.lon = doubleToDouble(lon);
	
	pMap->_gsPoint = Map_BLToGauss(pMap->_dPoint.lat, pMap->_dPoint.lon);
	pMap->gsPoint = Map_BLToGauss(pMap->dPoint.lat, pMap->dPoint.lon);
	
	pMap->a = DoubleSub(pMap->_gsPoint.lat, pMap->gsPoint.lat);
	pMap->b = DoubleSub(pMap->_gsPoint.lon, pMap->gsPoint.lon);

	dic = FastSqrtID(DoubleAdd(DoubleMul(pMap->a, pMap->a), DoubleMul(pMap->b, pMap->b)));
	pMap->c = DoubleDiv(intToDouble(1), dic);
	pMap->bl = DoubleMul(doubleToDouble(length), dic);
}

void Map_GetXY(Map* pMap, double lat, double lon, double *x, double *y)
{
	Double d, e, f, gf, g, h, i;
	DoublePoint gs = Map_BLToGauss(doubleToDouble(lat), doubleToDouble(lon));
	
	d = DoubleSub(gs.lon, pMap->gsPoint.lon);
	d = doubleToDouble(FastAbsD(DoubleTodouble(d)));
	gf = DoubleSub(gs.lat, pMap->gsPoint.lat);
	gf = doubleToDouble(FastAbsD(DoubleTodouble(gf)));
	//
	if (lon > DoubleTodouble(pMap->_dPoint.lon)){
		e = DoubleDiv(DoubleMul(d, pMap->c), pMap->b); //e = d * c / b
		f = DoubleDiv(DoubleMul(pMap->a, e), pMap->c); //f = a * e / c
		g = DoubleSub(gf, f);
		h = DoubleDiv(DoubleMul(pMap->a, g), pMap->c); //h = a * g / c
		i = DoubleDiv(DoubleMul(pMap->a, g), pMap->c); //i = b * g / c

		//lon mean X, lat mean Y
		//x = X - i * bl;
		//y = Y + (h + e) * bl;
		*x = DoubleTodouble(DoubleSub(pMap->X, DoubleMul(i, pMap->bl)));
		*y = DoubleTodouble(DoubleAdd(pMap->Y, DoubleMul(DoubleAdd(h, e), pMap->bl)));
	}
	else{
		e = DoubleDiv(DoubleMul(d, pMap->c), pMap->a); //e = d * c / a;
		f = DoubleDiv(DoubleMul(e, pMap->b), pMap->c); //f = e * b / c;
		
		g = DoubleSub(gf, f);
		h = DoubleDiv(DoubleMul(pMap->a, g), pMap->c); //h = a * g / c;
		i = DoubleDiv(DoubleMul(pMap->a, g), pMap->c); //i = b * g / c;

		//lon mean X, lat mean Y;
		//x = X - (e + i) * bl;
		//y = Y + h * bl;
		*x = DoubleTodouble(DoubleSub(pMap->X, DoubleMul(DoubleAdd(e, i), pMap->bl)));
		*y = DoubleTodouble(DoubleAdd(pMap->Y, DoubleMul(h, pMap->bl)));
	}
}
