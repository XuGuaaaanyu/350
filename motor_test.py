#Motor control library. 
#This library is used to control the speed and direction of the motors.

import RPi.GPIO as GPIO
import time 
import threading
from time import sleep
from Motor import Motor

GPIO.setmode(GPIO.BCM)


myMotor = Motor(20,21,18,19)

try:
    while True:
        myMotor.rotate(100)
        sleep(2)
        myMotor.stop()
        sleep(2)
        myMotor.rotate(-100)
        sleep(2)
        myMotor.stop()
        sleep(2)
        myMotor.rotate(50)
        sleep(2)
        myMotor.stop()
        sleep(2)
        myMotor.rotate(-50)
        sleep(2)
        myMotor.stop()
        sleep(2)

except KeyboardInterrupt:
    GPIO.cleanup()
    print('GPIO Good to Go')
