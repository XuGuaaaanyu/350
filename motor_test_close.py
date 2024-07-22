
import RPi.GPIO as GPIO
import time 
import threading
from time import sleep
from Motor import Motor

GPIO.setmode(GPIO.BCM)


myMotor = Motor(20,21,18,19)

Velocity_Kp = 20
Velocity_Ki = 30
myMotor.set_pi(Velocity_Kp,Velocity_Ki)

try:
    while True:
        myMotor.rotate_close(100)
        sleep(2)
        myMotor.stop()
        sleep(2)
        myMotor.rotate_close(-100)
        sleep(2)
        myMotor.stop()
        sleep(2)
        myMotor.rotate_close(50)
        sleep(2)
        myMotor.stop()
        sleep(2)
        myMotor.rotate_close(-50)
        sleep(2)
        myMotor.stop()
        sleep(2)

except KeyboardInterrupt:
    GPIO.cleanup()
    print('GPIO Good to Go')
