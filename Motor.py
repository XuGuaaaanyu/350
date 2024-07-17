#Motor control library. 
#This library is used to control the speed and direction of the motors.

import RPi.GPIO as GPIO
import time 
import threading
from time import sleep

GPIO.setmode(GPIO.BCM)

class Motor():
    def __init__(self,IN1,IN2,ENCODER,DIRECTION):
        #initialize a motor by specifying four pins 
        #IN1 and IN2 are pwm output to motor
        #ENCODER and DIRECTION are connected to encoder
        self.IN1 = IN1
        self.IN2 = IN2
        self.ENCODER = ENCODER
        self.DIRECTION = DIRECTION
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.ENCODER, GPIO.IN)
        GPIO.setup(self.DIRECTION, GPIO.IN)
        self.pwm1 = GPIO.PWM(self.IN1, 100)
        self.pwm2 = GPIO.PWM(self.IN2, 100)
        self.pwm1.start(0)
        self.pwm2.start(0)

        #attach interrupt to encoder
        GPIO.add_event_detect(self.ENCODER, GPIO.BOTH, callback=self.read_encoder)
        self.velocity = 0
        self.encoder_count = 0

        self.lock = threading.Lock()

        #start update_velocity function in a separate thread
        self.velocity_thread = threading.Thread(target=self.update_velocity)
        self.velocity_thread.daemon = True
        self.velocity_thread.start()
    
    def read_encoder(self, channel):
        with self.lock:
            if GPIO.input(self.ENCODER) == GPIO.LOW:
                if GPIO.input(self.DIRECTION) == GPIO.LOW:
                    self.encoder_count += 1
                else:
                    self.encoder_count -= 1
            else:
                if GPIO.input(self.DIRECTION) == GPIO.LOW:
                    self.encoder_count -= 1
                else:
                    self.encoder_count += 1
    
    def update_velocity(self):
        while True:
            with self.lock:
                self.velocity = self.encoder_count
                self.encoder_count = 0
            time.sleep(0.01)

    def get_velocity(self):
        with self.lock:
            return (self.velocity / 780.0) * 100 * 60
    


    def rotate(self, x = 70): 
        #Open loop control without the use of encoder
        #IN1: PWM, IN2: LOW, Forward rotation pwm, fast decay
        #IN1: LOW, IN2: PWM, Backward rotation pwm, fast decaY
        #default speed is 70%
        if  x > 0: #Forward rotation
            GPIO.output(self.IN2,GPIO.LOW)
            self.pwm1.ChangeDutyCycle(x) #set speed to x percent of max speed
        else:    #Backward rotation
            GPIO.output(self.IN1,GPIO.LOW)
            self.pwm2.ChangeDutyCycle(-x)

        print('Motor speed: ', x)
        print('Motor speed from encoder', self.get_velocity())



    def stop(self):
        GPIO.output(self.IN1,GPIO.LOW)
        GPIO.output(self.IN2,GPIO.LOW)
