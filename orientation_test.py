from mpu6050 import mpu6050
from time import sleep
import numpy as np

DELTA_T = 0.01 # sampling time

sensor = mpu6050.mpu6050(0x68)


def get_orientation():
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()

    AccX = accel_data['x']
    AccY = accel_data['y']
    AccZ = accel_data['z']

    GyroX = gyro_data['x']
    GyroY = gyro_data['y']
    GyroZ = gyro_data['z']

    acc_angle_x = np.arctan2(AccY, np.sqrt(AccX**2 + AccZ**2)) * 180 / np.pi    
    acc_angle_y = np.arctan2(AccX, np.sqrt(AccY**2 + AccZ**2)) * 180 / np.pi

    gyro_angle_x = GyroX * DELTA_T + gyro_angle_x
    gyro_angle_y = GyroY * DELTA_T + gyro_angle_y
    gyro_angle_z = GyroZ * DELTA_T + gyro_angle_z

    roll = 0.96 * gyro_angle_x + 0.04 * acc_angle_x
    pitch = 0.96 * gyro_angle_y + 0.04 * acc_angle_y
    yaw = gyro_angle_z

    return roll, pitch, yaw


try:
    while True:
        roll, pitch, yaw = get_orientation()
        print("Roll: ", roll, " Pitch: ", pitch, " Yaw: ", yaw)
        sleep(DELTA_T)
except KeyboardInterrupt:
    print("Terminated.")




