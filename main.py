#Modified
import time
import RPi.GPIO as GPIO
from mpu6050 import mpu6050
from Motor import Motor
import numpy as np

# GPIO pin setup
LEFT_MOTOR_IN1 = 20
LEFT_MOTOR_IN2 = 21
LEFT_MOTOR_ENCODER = 13
LEFT_MOTOR_DIRECTION = 19
RIGHT_MOTOR_IN1 = 12
RIGHT_MOTOR_IN2 = 16
RIGHT_MOTOR_ENCODER = 23
RIGHT_MOTOR_DIRECTION = 24
LEFT_CLUTCH = 18
RIGHT_CLUTCH = 25

GPIO.setmode(GPIO.BCM)

# Initialize MPU6050
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

# Initialize Motors
left_motor = Motor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_ENCODER, LEFT_MOTOR_DIRECTION)
right_motor = Motor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_ENCODER, RIGHT_MOTOR_DIRECTION)

# Initialize Clutches
GPIO.setup(LEFT_CLUTCH, GPIO.OUT)
GPIO.setup(RIGHT_CLUTCH, GPIO.OUT)


# PID parameters, controlling straight line and turning
Kp = 1.0
Ki = 0.0
Kd = 0.0

# Initialize PID variables
error = 0
last_error = 0
integral = 0
base_speed = 200
DELTA_T = 0.01 


def get_yaw():
    global gyro_angle_x, gyro_angle_y, gyro_angle_z
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
    return yaw


def run_car(yaw, target_yaw):
    global error, last_error, integral
    error = target_yaw - yaw  # Proportional term
    integral += error  # Integral term
    derivative = error - last_error  # Derivative term
    correction = Kp * error + Ki * integral + Kd * derivative  # PID output
    last_error = error  # Update last error

    left_speed = base_speed + correction
    right_speed = base_speed - correction

    set_motor_speed(left_speed, right_speed)

def camera_judgement():
    # Placeholder function for camera-based judgement
    # This function should return the target_yaw based on camera feedback
    # For example, it could return 0, 90, or -90
    # Here is a mock implementation
    return 0  # Replace with actual camera processing logic

def main():
    target_yaw = 0  # Initial target yaw for straight line movement

    try:
        while True:
            yaw = get_yaw()  # Measure the process value
            target_yaw = camera_judgement()  # Get the new target yaw from camera feedback
            run_car(yaw, target_yaw)

            # Print yaw and target yaw for debugging
            print(f"Yaw: {yaw:.2f}, Target Yaw: {target_yaw}")

            # Sleep for a short period to simulate a control loop
            time.sleep(DELTA_T)

    except KeyboardInterrupt:
        print("Program terminated.")
    finally:
        # Clean up GPIO
        GPIO.cleanup()

if __name__ == "__main__":
    main()