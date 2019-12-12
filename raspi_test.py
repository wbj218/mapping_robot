import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

left1=6
left2=5
leftpwm=4
#led1= 7
right1=13
right2=19
rightpwm=26

GPIO.setup(leftpwm, GPIO.OUT)
GPIO.setup(left1, GPIO.OUT)
GPIO.setup(left2, GPIO.OUT)
#GPIO.setup(led1, GPIO.OUT)
GPIO.setup(rightpwm, GPIO.OUT)
GPIO.setup(right1, GPIO.OUT)
GPIO.setup(right2, GPIO.OUT)
pwml= GPIO.PWM(leftpwm, 1000)
pwmr= GPIO.PWM(rightpwm,1000)

pwml.start(0)
pwmr.start(0)

def forward():
    pwml.ChangeDutyCycle(90)
    pwmr.ChangeDutyCycle(90)
    GPIO.output(left1, 1)
    GPIO.output(left2, 0)
    GPIO.output(right1,1)
    GPIO.output(right2,0)

def stop():
    pwml.ChangeDutyCycle(0)
    pwmr.ChangeDutyCycle(0)

    GPIO.output(left1, 0)
    GPIO.output(left2, 0)
    GPIO.output(right1,0)
    GPIO.output(right2,0)


i=0
while True:
    try:
        forward()
        time.sleep(1)
        #if i%2==0:
            #GPIO.output(led1, 1)
        #else:
            #GPIO.output(led1,0)

        #i+=1
    except KeyboardInterrupt:
        break

stop()
GPIO.cleanup()
