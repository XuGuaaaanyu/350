from mpu6050 import mpu6050
from time import sleep

sensor = mpu6050.mpu6050(0x68)

print("MPU6050 connection successful")

print ("gyro_range: " +  str(sensor.read_gyro_range()))
print ("accel_range: " +  str(sensor.read_accel_range()))

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



while True:
    accel_data = sensor.get_accel_data(g=True)
    gyro_data = sensor.get_gyro_data()


    #print("Accelerometer data") # accelerometer data in m/s^2
    #print("ax: " + str(accel_data['x']) + "\t ay: " + str(accel_data['y']) + "\t az: " + str(accel_data['z']))
    AccelX = (AccelX_k * accel_data['x'] + AccelX_b)
    AccelY = (AccelY_k * accel_data['y'] + AccelY_b)
    AccelZ = (AccelZ_k * accel_data['z'] + AccelZ_b)
    print("AccelX: " + str(AccelX) + "\t AccelY: " + str(AccelY) + "\t AccelZ: " + str(AccelZ))

    #print("Gyroscope data") # gyroscope data in degree/s
    #print("x: " + str(gyro_data['x']) + "\t y: " + str(gyro_data['y']) + "\t z: " + str(gyro_data['z']))
    GyroX = gyro_data['x'] - GyroX_offset
    GyroY = gyro_data['y'] - GyroY_offset
    GyroZ = gyro_data['z'] - GyroZ_offset
    #print("GyroX: " + str(GyroX) + "\t GyroY: " + str(GyroY) + "\t GyroZ: " + str(GyroZ))


    sleep(0.1)
