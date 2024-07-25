#output the roll, yaw, and pitch from mpu6050 sensor

from mpu6050 import mpu6050
from time import sleep
import numpy as np

DELTA_T = 0.01 # sampling time

sensor = mpu6050.mpu6050(0x68)

#Gyroscope calibration constants
GyroX_offset = - 2.375541984732824
GyroY_offset = - 7.537664122137405
GyroZ_offset = - 1.764442748091603

#Accelerometer calibration constants
AccelX_k = 0.99873091
AccelX_b = -0.04609702
AccelY_k = 0.99933942
AccelY_b = 0.00491675
AccelZ_k = 0.96768578
AccelZ_b = -0.02067255


gyro_angle_x = 0
gyro_angle_y = 0
gyro_angle_z = 0

try:
    while True:
        accel_data = sensor.get_accel_data(g=True)
        gyro_data = sensor.get_gyro_data()

        AccX = (AccelX_k * accel_data['x'] + AccelX_b)
        AccY = (AccelY_k * accel_data['y'] + AccelY_b)
        AccZ = (AccelZ_k * accel_data['z'] + AccelZ_b)

        GyroX = gyro_data['x'] - GyroX_offset
        GyroY = gyro_data['y'] - GyroY_offset
        GyroZ = gyro_data['z'] - GyroZ_offset

        acc_angle_x = np.arctan2(AccY, np.sqrt(AccX**2 + AccZ**2)) * 180 / np.pi    
        acc_angle_y = np.arctan2(AccX, np.sqrt(AccY**2 + AccZ**2)) * 180 / np.pi

        gyro_angle_x += GyroX * DELTA_T 
        gyro_angle_y += GyroY * DELTA_T 
        gyro_angle_z += GyroZ * DELTA_T

        roll = 0.96 * gyro_angle_x + 0.04 * acc_angle_x
        pitch = 0.96 * gyro_angle_y + 0.04 * acc_angle_y
        yaw = gyro_angle_z
        print("Roll: ", roll, " Pitch: ", pitch, " Yaw: ", yaw)
        sleep(DELTA_T)
except KeyboardInterrupt:
    print("Terminated.")




