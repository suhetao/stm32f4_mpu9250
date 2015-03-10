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

#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "PID.h"

//Modelling, Identification and Control of a Quadrotor Helicopter

#define PI (3.1415926535897932384626433832795f)
#define PI_6 (0.52359877559829887307710723054658f)
#define MIN_ANG (-PI_6)
#define MAX_ANG PI_6

typedef struct CONTROLLERPARAMETER_T
{
	float m; //mass of the quadrotor
	float Ixx; //body moment of inertia around the x-axis
	float Iyy; //body moment of inertia around the y-axis
	float Izz; //body moment of inertia around the z-axis
	float I[9]; //body inertia matrix
	float d; //drag factor
	float b; //thrust factor
	float l; //center of quadrotor to center of propeller distance
	
	float X;
	float Y;
	float Z;
	PIDController PostionX;
	PIDController PostionY;
	PIDController PostionZ;
	
	// not use yet
	float g; //acceleration due to gravity
	float n; //number of data acquired
	float N; //gear box reduction ratio
	float h; //PWM code vector
	float R; //motor resistance
	float Jtp; //total rotational moment of inertia around the propeller axis
	float Ke; //electric motor constant
	float Km; //mechanic motor constant
	float L; //motor inductance
	
}QuadrotorParameter;

typedef enum TRAJECTORY_T{
	X = 0,
	Y = 1,
	Z = 2,
	PSI = 3
}Trajectory;


typedef enum EULER_T{
	ROLL = 0,
	PITCH = 1,
	YAW = 2
}Euler;

void EulerConv(float* dt, float *deta);
void TorqueConv(float *eta, float *deta, float *dt);
void ForceConv(float *eta, float dz, float *df);
void TorqueInv(float *dt, float df, float *domega);

#endif
