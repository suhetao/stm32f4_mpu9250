# stm32f4_mpu9250
Access the data of 3-axis magnetometer and DMP from MPU9250 with SPI interface 

All data fusion (including the data of dmp output, such as the accelerometer data,
gyroscope, 6-axis quaternion and internal magnetometer data) via a 7-state, 13-mesurement
EKF(Extended Kalman filter) / Unscented Kalman Filter(UKF) / Cubature Kalman Filters (CKF) Algorithm.

1.kalman feature:

	prediction state: q0 q1 q2 q3 wx wy wz
	mesurement:q0 q1 q2 q3(q0~q3 from dmp ouput) ax ay az wx wy wz mx my mz

2.Add a miniIMU for doctor's miniQuadrotor
	
	add a new fixed-point ekf algorithm for doctor's miniQuadrotor 
	please check the miniIMU directory at the root and look the usage.txt for using!

3.Add a win32 application for serial port communication

	calibraion for accelerometer, gyroscope, magnetometer
	please check the "Calibration App" directory at the root

4.Add a miniAHRS for 9-axis fusion, such as ax ay az wx wy wz mx my mz form accelerometer,gyroscope,magnetometer

	7-state ekf algorithm: quaternion and 3-axis gyroscope bais
	3-mesurement for accelerometer
	3-mesurement for magnetometer

5.Other stuff

	using 4-order Runge_Kutta to slove the quaternion differential equation.