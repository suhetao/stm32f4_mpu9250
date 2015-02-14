# stm32f4_mpu9250
Access the data of 3-axis magnetometer and DMP from MPU9250 with SPI interface 

All data fusion (including the data of dmp output, such as the accelerometer data,
gyroscope, 6-axis quaternion and internal magnetometer data) via a 7-state, 13-mesurement
EKF(Extended Kalman filter) / Unscented Kalman Filter(UKF) / Cubature Kalman Filters (CKF) Algorithm.

1.kalman feature:
	prediction state: q0 q1 q2 q3 wx wy wz
	
	mesurement:q0 q1 q2 q3(q0~q3 from dmp ouput) ax ay az wx wy wz mx my mz

2.Add a miniIMU for doctor's miniQuadrotor
	
	2.1add a new fixed-point ekf algorithm for doctor's miniQuadrotor 
	
please check the miniIMU directory at the root and look the usage.txt for using!
