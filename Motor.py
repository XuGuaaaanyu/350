#Motor control library. 
#This library is used to control the speed and direction of the motors.

import RPi.GPIO as GPIO
import time 
import threading
from time import sleep

GPIO.setmode(GPIO.BCM)

class Motor():
    def __init__(self,IN1,IN2,ENCODER,DIRECTION,Kp=1.0,Ki=0.0):
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
        self.pwm1 = GPIO.PWM(self.IN1, 50)
        self.pwm2 = GPIO.PWM(self.IN2, 50)
        self.pwm1.start(0)
        self.pwm2.start(0)

        self.Kp = Kp
        self.Ki = Ki

        self.contro_mode = 'open'

        #attach interrupt to encoder
        GPIO.add_event_detect(self.ENCODER, GPIO.BOTH, callback=self.read_encoder)
        self.velocity = 0
        self.encoder_count = 0

        self.lock = threading.Lock()

        #start update_velocity function in a separate thread
        self.velocity_thread = threading.Thread(target=self.update_velocity)
        self.velocity_thread.daemon = True
        self.velocity_thread.start()

        #start closed loop control in a separate thread
        self.target_velocity = 0
        self.control_thread = threading.Thread(target=self.control)
        self.control_thread.daemon = True
        self.control_thread.start()
    
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
    


    def rotate_open(self, x): 
        self.control_mode = 'open'
        #Open loop control without the use of encoder
        #IN1: PWM, IN2: LOW, Forward rotation pwm, fast decay
        #IN1: LOW, IN2: PWM, Backward rotation pwm, fast decaY
        if  x > 0: #Forward rotation
            GPIO.output(self.IN2,GPIO.LOW)
            self.pwm1.ChangeDutyCycle(x) #set speed to x percent of max speed
            self.pwm2.ChangeDutyCycle(0)
        else:    #Backward rotation
            GPIO.output(self.IN1,GPIO.LOW)
            self.pwm2.ChangeDutyCycle(-x)
            self.pwm1.ChangeDutyCycle(0)

        print('Motor speed: ', x)
        print('Motor speed from encoder', self.get_velocity())
    
    def control(self):
        while True:
            if self.contro_mode == 'close':
                current_velocity = self.get_velocity()
                error = self.target_velocity - current_velocity
                self.integral += error
                control_signal = self.Kp * error + self.Ki * self.integral
                control_signal = max(min(control_signal, 100), -100)

                if control_signal >= 0:
                    GPIO.output(self.IN2, GPIO.HIGH)
                    self.pwm1.ChangeDutyCycle(control_signal)
                    self.pwm2.ChangeDutyCycle(0)
                else:
                    GPIO.output(self.IN1, GPIO.HIGH)
                    self.pwm2.ChangeDutyCycle(-control_signal)
                    self.pwm1.ChangeDutyCycle(0)
                
                print('Control signal: ', control_signal)
                print('Current velocity: ', current_velocity)
            
            time.sleep(0.01)
    
    def rotate_close(self, target_velocity):
        self.contro_mode = 'close'
        self.target_velocity = target_velocity
        self.integral = 0


    def stop(self):
        self.contro_mode = 'open'
        #stop the motor
        GPIO.output(self.IN1,GPIO.LOW)
        GPIO.output(self.IN2,GPIO.LOW)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)

    def set_pi (self,Kp,Ki):
        self.Kp = Kp
        self.Ki = Ki