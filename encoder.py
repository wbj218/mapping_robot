import RPi.GPIO as GPIO
import time
import math
import board
import busio
import time
import adafruit_lsm9ds1

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)


dist_meas = 0.00
km_h = 0
elapse = 0
encoder = 12
pulse = 0

def init_GPIO():
#    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(encoder,GPIO.IN,GPIO.PUD_UP)
    
def calculate_elapse(channel):
    global pulse,start_timer,elapse
    pulse+=1
    elapse = time.time() - start_timer
    start_timer = time.time()

def calculate_speed(r_cm):
    global pulse,elapse,rpm,dist_meas,km_h,km_s
    if elapse !=0:
        rpm=1/elapse *60
        circ_cm = (2*math.pi*r_cm)
        dist_km = circ_cm/10000
        km_s = dist_km/elapse
        km_h = km_s *3600
        dist_meas = (dist_km*pulse)*1000
        return km_h

def init_interrupt():
    GPIO.add_event_detect(encoder,GPIO.FALLING,callback = calculate_elapse,bouncetime = 20)
    
start_timer=time.time()

init_GPIO()
init_interrupt()

while True:
    try:
        calculate_speed(3.2)
        print('pulse',pulse,'speed',km_h)
        time.sleep(2)
        
    except KeyboardInterrupt:
        GPIO.cleanup()
        break
        
