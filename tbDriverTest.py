import RPi.GPIO as GPIO
import time

pwma=4
ain2=6
ain1=5

bin1=13
bin2=19
pwmb=26

standby=17

GPIO.setmode(GPIO.BCM)

GPIO.setup(ain1,GPIO.OUT)
GPIO.setup(ain2,GPIO.OUT)
GPIO.setup(pwma,GPIO.OUT)
GPIO.setup(bin1,GPIO.OUT)
GPIO.setup(bin2,GPIO.OUT)
GPIO.setup(pwmb,GPIO.OUT)
GPIO.setup(standby,GPIO.OUT)
GPIO.output(standby,GPIO.HIGH)

pwml=GPIO.PWM(pwma, 1000)
pwmr=GPIO.PWM(pwmb, 1000)

pwml.start(0)
pwmr.start(0)

def forward():
    pwml.ChangeDutyCycle(40)
    pwmr.ChangeDutyCycle(40)
    GPIO.output(ain1, 1)
    GPIO.output(ain2, 0)
    GPIO.output(bin1, 1)
    GPIO.output(bin2, 0)

def stop():
    pwml.ChangeDutyCycle(0)
    pwmr.ChangeDutyCycle(0)
    GPIO.output(ain1, 1)
    GPIO.output(ain2, 1)
    GPIO.output(bin1, 1)
    GPIO.output(bin2, 1)

while True:
    try:
        forward()
        time.sleep(1)
        #stop()
        #time.sleep(1)
    except KeyboardInterrupt:
        break


stop()
GPIO.cleanup()

