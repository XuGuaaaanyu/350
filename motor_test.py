from time import sleep
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

class Motor():
    def __init__(self,IN1,IN2):
        self.IN1 = IN1
        self.IN2 = IN2
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        self.pwm1 = GPIO.PWM(self.IN1, 100)
        self.pwm2 = GPIO.PWM(self.IN2, 100)
        self.pwm1.start(0)
        self.pwm2.start(0)
    
    #IN1: PWM, IN2: LOW, Forward rotation pwm, fast decay
    #IN1: LOW, IN2: PWM, Backward rotation pwm, fast decaY
    def rotate(self, x = 70): #default speed is 70%
        if  x > 0: #Forward rotation
            GPIO.output(self.IN2,GPIO.LOW)
            self.pwm1.ChangeDutyCycle(x) #set speed to x percent of max speed
        else:    #Backward rotation
            GPIO.output(self.IN1,GPIO.LOW)
            self.pwm2.ChangeDutyCycle(-x)
    
    def stop(self):
        GPIO.output(self.IN1,GPIO.LOW)
        GPIO.output(self.IN2,GPIO.LOW)



myMotor = Motor(20,21)

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
