import RPi.GPIO as GPIO
from time import sleep
from Motor import Motor

#Control strategy:
#small forward:     wheel clutch: on;       car clutch: off;        motor: +
#Expansion:         wheel clutch: off;      car clutch: on;         motor: +
#Large forward:     wheel clutch: on;       car clutch: off;        motor: +
#Shrink:            wheel clutch: off;      car clutch: on;         motor: -


def expand(left_motor, right_motor, left_clutch, right_clutch):
    #Wheel Expansion
    GPIO.output(left_clutch, GPIO.HIGH)
    GPIO.output(right_clutch, GPIO.HIGH)
    left_motor.rotate_open(-80)
    right_motor.rotate_open(80) #check this direction carefully
    sleep(2) #time for expansion, could be modified
    left_motor.stop()
    right_motor.stop()
    sleep(0.5)
    GPIO.output(left_clutch, GPIO.LOW)
    GPIO.output(right_clutch, GPIO.LOW)

def shrink(left_motor, right_motor, left_clutch, right_clutch):
    #Wheel Shrink
    GPIO.output(left_clutch, GPIO.HIGH)
    GPIO.output(right_clutch, GPIO.HIGH)
    left_motor.rotate_open(80)
    right_motor.rotate_open(-80)
    sleep(2) #time for shrink, could be modified
    left_motor.stop()
    right_motor.stop()
    sleep(0.5)
    GPIO.output(left_clutch, GPIO.LOW)
    GPIO.output(right_clutch, GPIO.LOW)


