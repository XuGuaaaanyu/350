import RPi.GPIO as GPIO
from time import sleep
from Motor import Motor

#Contro strategy:
#small forward:     wheel clutch: on;       car clutch: off;        motor: +
#Expansion:         wheel clutch: off;      car clutch: on;         motor: +
#Large forward:     wheel clutch: on;       car clutch: off;        motor: +
#Shrink:            wheel clutch: off;      car clutch: on;         motor: -

GPIO.setmode(GPIO.BCM)


left_motor = Motor(20,21,23,24)
right_motor = Motor(12,16,13,19)

left_clutch = 18
right_clutch = 26

GPIO.setup(left_clutch, GPIO.OUT)
GPIO.setup(right_clutch, GPIO.OUT)


def expand(left_motor, right_motor, left_clutch, right_clutch):
    #Wheel Expansion
    left_motor.stop()
    right_motor.stop()
    sleep(0.5)
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
    left_motor.stop()
    right_motor.stop()
    sleep(0.5)
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

try:
    while True:
        left_motor.rotate_open(100)
        right_motor.rotate_open(100)
        sleep(2)
        expand(left_motor, right_motor, left_clutch, right_clutch)
        left_motor.rotate_open(100)
        right_motor.rotate_open(100)
        sleep(2)
        shrink(left_motor, right_motor, left_clutch, right_clutch)
        left_motor.rotate_open(100)
        right_motor.rotate_open(100)
        sleep(2)
except KeyboardInterrupt:
    GPIO.cleanup()
    print('GPIO Good to Go')
