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

#ifndef _MAP_H
#define _MAP_H

#include "stm32f4xx.h"

#include "Double.h"

typedef struct DOUBLEPOINT_T
{
	Double lat;
	Double lon;
}DoublePoint;

typedef struct MAP_T
{
	//reference point
	DoublePoint _dPoint;
	DoublePoint dPoint;
	
	DoublePoint _gsPoint;
	DoublePoint gsPoint;
	//
	//reference position
	Double X, Y;
	//
	//coefficient
	Double a, b, c, bl;
	//
	
}	Map;

//////////////////////////////////////////////////////////////////////////
//must init with local two reference points including lat, lon and the length
//betweeb two reference points 
void Map_Init(Map* pMap, double x1, double y1, double x2, double y2,
	double x, double y, double length);
void Map_GetXY(Map* pMap, double lat, double lon, double *x, double *y);

#endif
