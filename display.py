import smbus
import math
import pygame
import numpy as np
from pygame.locals import *

from mpu6050 import mpu6050
from time import sleep

sensor = mpu6050.mpu6050(0x68)

# Initialize pygame
pygame.init()

# Set up screen size
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('IMU Cube Visualization')

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

# IMU Parameters
MPU6050_ADDR = 0x68  # MPU6050 I2C address

# Constants
ACCEL_SCALE = 16384.0  # Scale factor for accelerometer (sensitivity based on full-scale range)
GYRO_SCALE = 131.0     # Scale factor for gyroscope (sensitivity based on full-scale range)

DELTA_T = 0.01  # Sampling time (s)
# Variables for orientation
roll = 0.0
pitch = 0.0
yaw = 0.0

gyro_angle_x = 0.0
gyro_angle_y = 0.0
gyro_angle_z = 0.0

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

# Initialize the SMBus interface for I2C communication
bus = smbus.SMBus(1)


def calculate_orientation(accel_data, gyro_data):
    global roll, pitch, yaw
    global gyro_angle_x, gyro_angle_y, gyro_angle_z
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

def draw_cube(screen, roll, pitch, yaw):
    cube_size = 100
    cube_center = (WIDTH // 2, HEIGHT // 2)
    cube_color = WHITE

    # Calculate cube vertices based on orientation
    vertices = [
        (cube_center[0] + cube_size * math.cos(roll) * math.cos(yaw) - cube_size * math.sin(roll) * math.sin(yaw),
         cube_center[1] + cube_size * math.cos(pitch) * math.sin(yaw)),
        (cube_center[0] - cube_size * math.cos(roll) * math.cos(yaw) - cube_size * math.sin(roll) * math.sin(yaw),
         cube_center[1] + cube_size * math.cos(pitch) * math.sin(yaw)),
        (cube_center[0] + cube_size * math.cos(roll) * math.cos(yaw) + cube_size * math.sin(roll) * math.sin(yaw),
         cube_center[1] - cube_size * math.cos(pitch) * math.sin(yaw)),
        (cube_center[0] - cube_size * math.cos(roll) * math.cos(yaw) + cube_size * math.sin(roll) * math.sin(yaw),
         cube_center[1] - cube_size * math.cos(pitch) * math.sin(yaw))
    ]

    # Draw cube
    pygame.draw.polygon(screen, cube_color, vertices)

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False

    # Read sensor data
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()

    # Calculate orientation
    calculate_orientation(accel_data,gyro_data)

    # Clear screen
    screen.fill(BLACK)

    # Draw cube with current orientation
    draw_cube(screen, roll, pitch, yaw)

    # Update display
    pygame.display.flip()

pygame.quit()
