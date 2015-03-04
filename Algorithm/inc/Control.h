#ifndef _CONTROL_H_
#define _CONTROL_H_

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
	float Jtp; //total rotational moment of inertia around the propeller axis
	float Ke; //electric motor constant
	float Km; //mechanic motor constant
	float L; //motor inductance
	float b; //thrust factor
	float l; //center of quadrotor to center of propeller distance
	float g; //acceleration due to gravity
	float n; //number of data acquired
	float N; //gear box reduction ratio
	float h; //PWM code vector
	float R; //motor resistance
	float d; //drag factor
}ControllerParameter;

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
void TorqueConv(float *eta, float *deta, float *I, float *dt);
void ForceConv(float *eta, float dz, float m, float *df);
void TorqueInv(float *dt, float df, float b, float d, float l, float *domega);
void QuadrotorDynamics(float* dv, float *eta, float *deta, float *p, float *dp, float *omega);

#endif
