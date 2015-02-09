# stm32f4_mpu9250
Access the data of 3-axis magnetometer and DMP from MPU9250 with SPI interface 

All data fusion (including the data of dmp output, such as the accelerometer data,
gyroscope, 6-axis quaternion and internal magnetometer data) via a 7-state, 13-mesurement
EKF(Extended Kalman filter) / Unscented Kalman Filter(UKF) Algorithm.

TODO:
1. If I have more free time,I will implement Cubature Kalman filter (CKF) as soon as possible.