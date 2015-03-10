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

#ifndef _PID_H_
#define _PID_H_

typedef struct PIDCONTROLLER_T
{
	float A0; // < The derived gain, A0 = Kp + Ki + Kd .
	float A1; // < The derived gain, A1 = -Kp - 2Kd.
	float A2; // < The derived gain, A2 = Kd .
	float state[3]; // < The state array of length 3.
	float Kp; // < The proportional gain.
	float Ki; // < The integral gain.
	float Kd; // < The derivative gain.
} PIDController;

__inline float PID_Calculate(PIDController *S, float in)
{
	float out;
	// y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2] 
	out = (S->A0 * in) + (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);

	// Update state
	S->state[1] = S->state[0];
	S->state[0] = in;
	S->state[2] = out;

	// return to application
	return (out);
}

__inline void PID_Init(PIDController *S)
{
	//Derived coefficient A0
	S->A0 = S->Kp + S->Ki + S->Kd;

	//Derived coefficient A1
	S->A1 = (-S->Kp) - ((float) 2.0 * S->Kd);

	//Derived coefficient A2
	S->A2 = S->Kd;

	S->state[0] = 0.0f;
	S->state[1] = 0.0f;
	S->state[2] = 0.0f;
}

__inline void PID_Reset(PIDController *S)
{
	S->state[0] = 0.0f;
	S->state[1] = 0.0f;
	S->state[2] = 0.0f;
}

#endif
