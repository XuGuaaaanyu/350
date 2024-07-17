import RPi.GPIO as GPIO
import time
import spidev
from threading import Thread

# Pin Definitions
BIN1 = 5
BIN2 = 6
AIN1 = 9
AIN2 = 10
VOLTAGE = 5  # ADC channel
ENCODER_L = 8
DIRECTION_L = 4
ENCODER_R = 7
DIRECTION_R = 2
LED = 13

# Global Variables
V = 0
TimeA = 0
EncoderTime = 0
EncoderFlag = True
Mode = False
LED_Count = 0
Velocity_L = 0
Velocity_R = 0
Velocity_Left = 0
Velocity_Right = 0
delayShow = 0
TimeB = 0
MotorFlag = True
TimeFlag = True
putPWM = 128

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)
GPIO.setup(LED, GPIO.OUT)
GPIO.setup(ENCODER_L, GPIO.IN)
GPIO.setup(DIRECTION_L, GPIO.IN)
GPIO.setup(ENCODER_R, GPIO.IN)
GPIO.setup(DIRECTION_R, GPIO.IN)

# Initialize PWM
pwm_AIN1 = GPIO.PWM(AIN1, 1000)
pwm_AIN2 = GPIO.PWM(AIN2, 1000)
pwm_BIN1 = GPIO.PWM(BIN1, 1000)
pwm_BIN2 = GPIO.PWM(BIN2, 1000)
pwm_AIN1.start(0)
pwm_AIN2.start(0)
pwm_BIN1.start(0)
pwm_BIN2.start(0)

# Initialize SPI for ADC
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000

def read_adc(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

def set_pwm_a(pwm):
    if pwm > 0:
        pwm_AIN1.ChangeDutyCycle(100)
        pwm_AIN2.ChangeDutyCycle(100 - pwm)
    else:
        pwm_AIN2.ChangeDutyCycle(100)
        pwm_AIN1.ChangeDutyCycle(100 + pwm)

def set_pwm_b(pwm):
    if pwm > 0:
        pwm_BIN1.ChangeDutyCycle(100)
        pwm_BIN2.ChangeDutyCycle(100 - pwm)
    else:
        pwm_BIN2.ChangeDutyCycle(100)
        pwm_BIN1.ChangeDutyCycle(100 + pwm)

def read_encoder_l(channel):
    global Velocity_L
    if GPIO.input(ENCODER_L) == GPIO.LOW:
        if GPIO.input(DIRECTION_L) == GPIO.LOW:
            Velocity_L -= 1
        else:
            Velocity_L += 1
    else:
        if GPIO.input(DIRECTION_L) == GPIO.LOW:
            Velocity_L += 1
        else:
            Velocity_L -= 1

def read_encoder_r(channel):
    global Velocity_R
    if GPIO.input(ENCODER_R) == GPIO.LOW:
        if GPIO.input(DIRECTION_R) == GPIO.LOW:
            Velocity_R += 1
        else:
            Velocity_R -= 1
    else:
        if GPIO.input(DIRECTION_R) == GPIO.LOW:
            Velocity_R -= 1
        else:
            Velocity_R += 1

# Attach Interrupts
GPIO.add_event_detect(ENCODER_L, GPIO.BOTH, callback=read_encoder_l)
GPIO.add_event_detect(ENCODER_R, GPIO.BOTH, callback=read_encoder_r)

def main_loop():
    global TimeA, EncoderTime, EncoderFlag, TimeFlag, TimeB, MotorFlag, LED_Count, Velocity_L, Velocity_R, Velocity_Left, Velocity_Right, delayShow, putPWM

    while True:
        TimeA = int(round(time.time() * 1000))

        if EncoderFlag:
            EncoderFlag = False
            EncoderTime = TimeA

        if TimeFlag:
            TimeFlag = False
            TimeB = TimeA

        # Change PWM based on Serial input (Not implemented here)
        # This part will require a different mechanism to capture serial input, e.g., through a specific protocol or interface.

        # Change motor direction every 4 seconds
        if TimeA - TimeB > 3999:
            TimeFlag = True
            MotorFlag = not MotorFlag

        if MotorFlag:
            set_pwm_a(putPWM)
            set_pwm_b(putPWM)
        else:
            set_pwm_a(-putPWM)
            set_pwm_b(-putPWM)

        # Blink LED every second
        if LED_Count == 99:
            LED_Count = 0
            Mode = not Mode
            GPIO.output(LED, Mode)

        # Read encoder data every 10ms
        if TimeA - EncoderTime > 9:
            EncoderFlag = True
            Velocity_Left = Velocity_L
            Velocity_L = 0
            Velocity_Right = Velocity_R
            Velocity_R = 0

            Velocity_Left = (Velocity_Left / 780.0) * 100 * 60
            Velocity_Right = (Velocity_Right / 780.0) * 100 * 60

            delayShow += 1
            LED_Count += 1

            # Display data every 50ms
            if delayShow == 50:
                delayShow = 0
                print(f"Velocity_L = {Velocity_Left} 转/分")
                print(f"Velocity_R = {Velocity_Right} 转/分")

                V = read_adc(VOLTAGE)
                print(f"输入电压 = {V * 0.05371}V")

                print(f"PWM = {putPWM}")
                print("\n\n")

        time.sleep(0.01)

if __name__ == "__main__":
    try:
        main_loop()
    except KeyboardInterrupt:
        pwm_AIN1.stop()
        pwm_AIN2.stop()
        pwm_BIN1.stop()
        pwm_BIN2.stop()
        GPIO.cleanup()
