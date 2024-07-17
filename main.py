#Written by GPT, to be modified.

import time
import math
import RPi.GPIO as GPIO
from mpu6050 import mpu6050

# GPIO pin setup
MOTOR_LEFT_PWM_PIN = 17
MOTOR_LEFT_DIR_PIN = 27
MOTOR_RIGHT_PWM_PIN = 22
MOTOR_RIGHT_DIR_PIN = 23

# Initialize MPU6050
mpu = mpu6050(0x68)

# PID parameters
Kp = 1.0
Ki = 0.0
Kd = 0.0

# Initialize PID variables
error = 0
last_error = 0
integral = 0
base_speed = 200

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LEFT_PWM_PIN, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_DIR_PIN, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_PWM_PIN, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_DIR_PIN, GPIO.OUT)

# Set up PWM channels
pwm_motor_left = GPIO.PWM(MOTOR_LEFT_PWM_PIN, 100)  # 100 Hz
pwm_motor_right = GPIO.PWM(MOTOR_RIGHT_PWM_PIN, 100)  # 100 Hz

# Start PWM with 0% duty cycle
pwm_motor_left.start(0)
pwm_motor_right.start(0)

def get_yaw():
    gyro_data = mpu.get_gyro_data()
    yaw = gyro_data['z']
    return yaw

def set_motor_speed(left_speed, right_speed):
    # Constrain speed to 0-100% for PWM
    left_speed = max(min(left_speed, 100), -100)
    right_speed = max(min(right_speed, 100), -100)

    # Set direction for motor A
    if left_speed >= 0:
        GPIO.output(MOTOR_LEFT_DIR_PIN, GPIO.HIGH)
    else:
        GPIO.output(MOTOR_LEFT_DIR_PIN, GPIO.LOW)
        left_speed = -left_speed  # Make speed positive for PWM

    # Set direction for motor B
    if right_speed >= 0:
        GPIO.output(MOTOR_RIGHT_DIR_PIN, GPIO.HIGH)
    else:
        GPIO.output(MOTOR_RIGHT_DIR_PIN, GPIO.LOW)
        right_speed = -right_speed  # Make speed positive for PWM

    # Set PWM duty cycle
    pwm_motor_left.ChangeDutyCycle(left_speed)
    pwm_motor_right.ChangeDutyCycle(right_speed)

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
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Program terminated.")
    finally:
        # Clean up GPIO
        pwm_motor_left.stop()
        pwm_motor_right.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()