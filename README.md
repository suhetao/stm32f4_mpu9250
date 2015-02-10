# stm32f4_mpu9250
Access the data of 3-axis magnetometer and DMP from MPU9250 with SPI interface 

All data fusion (including the data of dmp output, such as the accelerometer data,
gyroscope, 6-axis quaternion and internal magnetometer data) via a 7-state, 13-mesurement
EKF(Extended Kalman filter) / Unscented Kalman Filter(UKF) / Cubature Kalman Filters (CKF) Algorithm.

feature

prediction state: q0 q1 q2 q3 wx wy wz

mesurement:q0 q1 q2 q3 ax ay az wx wy wz mx my mzv (q0~q3 from dmp ouput)