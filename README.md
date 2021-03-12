# mpu6050_DMP_python
attempt to port DMP module library to raspberry pico 

based on:
https://github.com/thisisG/MPU6050-I2C-Python-Class


This is an experiment to port existing librarty from python to micropython raspberry pi board. 

DMP should in theory provide better datat than reading from Gyro and accel registers directly. 
It also uses FIFO. 

