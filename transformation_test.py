import RPi.GPIO as GPIO
import time 
from Motor import Motor

#Contro strategy:
#small forward:     wheel clutch: on;       car clutch: off;        motor: +
#Expansion:         wheel clutch: off;      car clutch: on;         motor: +
#Large forward:     wheel clutch: on;       car clutch: off;        motor: +
#Shrink:            wheel clutch: off;      car clutch: on;         motor: -

#initialize motor
myMotor = Motor(20,21,18,19)

GPIO.setmode(GPIO.BCM)

wheel_clutch = 16
car_clutch = 12

GPIO.setup(wheel_clutch, GPIO.OUT)
GPIO.setup(car_clutch, GPIO.OUT)

def small_forward():
    myMotor.rotate_close(100)
    GPIO.output(wheel_clutch, GPIO.HIGH)
    GPIO.output(car_clutch, GPIO.LOW)


def expand():
    myMotor.rotate_close(60)
    GPIO.output(wheel_clutch, GPIO.LOW)
    GPIO.output(car_clutch, GPIO.HIGH)
    time.sleep(1)

def large_forward():
    myMotor.rotate_close(100)
    GPIO.output(wheel_clutch, GPIO.HIGH)
    GPIO.output(car_clutch, GPIO.LOW)

def shrink():
    myMotor.rotate_close(-60)
    GPIO.output(wheel_clutch, GPIO.LOW)
    GPIO.output(car_clutch, GPIO.HIGH)
    time.sleep(1)

