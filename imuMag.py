import RPi.GPIO as GPIO
import time
import math
import board
import busio
import time
import adafruit_lsm9ds1
from madgwickahrs import MadgwickAHRS

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

accel_x, accel_y, accel_z = sensor.acceleration
gyro_x, gyro_y, gyro_z = sensor.gyro
mag_x, mag_y, mag_z = sensor.magnetic

a_x=0
a_y=0
a_z=0
g_x=0
g_y=0
g_z=0
m_x=0
m_y=0
m_z=0

for i in range(100):
    accel_x, accel_y, accel_z = sensor.acceleration
    gyro_x, gyro_y, gyro_z = sensor.gyro
    mag_x, mag_y, mag_z = sensor.magnetic
    a_x += accel_x
    a_y += accel_y
    a_z += accel_z
    g_x += gyro_x
    g_y += gyro_y
    g_z += gyro_z
    m_x += mag_x
    m_y += mag_y
    m_z += mag_z
    time.sleep(0.05)

a_x=a_x/100
a_y=a_y/100
a_z=a_z/100
a_z=a_z+9.81
g_x=g_x/100
g_y=g_y/100
g_z=g_z/100
m_x=m_x/100
m_y=m_y/100
m_z=m_z/100

heading = MadgwickAHRS()
while True:
    try:
        accel_x, accel_y, accel_z = sensor.acceleration
        gyro_x, gyro_y, gyro_z = sensor.gyro
        mag_x, mag_y, mag_z = sensor.magnetic
        acc=[accel_x-a_x, accel_y-a_y, accel_z-a_z]
        gyro=[gyro_x-g_x, gyro_y-g_y, gyro_z-g_z]
        mag=[mag_x-m_x, mag_y-m_y, mag_z-m_z]
        heading.update(gyro, acc, mag)
        ahrs = heading.quaternion.to_euler_angles()
        #roll = ahrs[0]
        #pitch = ahrs[1]
        yaw = ahrs[2]
        print(yaw)
        time.sleep(0.05)
    except KeyboardInterrupt:
        GPIO.cleanup()
        break






    
