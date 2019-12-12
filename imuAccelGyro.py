import RPi.GPIO as GPIO
import time
import math
import board
import busio
import time
import adafruit_lsm9ds1

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# IMU calibration
y_sum=0
z_sum=0

accel_x_sum=0
accel_z_sum=0

for i in range(100):
    accel_x, accel_y, accel_z = sensor.accel
    gyro_x, gyro_y, gyro_z = sensor.gyro
    accel_x_sum += accel_x
    accel_z_sum += accel_z
    y_sum += gyro_y
    z_sum += gyro_z
    time.sleep(0.05)

avg_y=y_sum/100
avg_z=z_sum/100

avg_accelx = accel_x_sum/100
avg_accelz = accel_z_sum/100 + 9.81

#low pass filter parameters
w=2
T=0.01

#get first two readings
accel_x, accel_y, accel_z = sensor.accel
gyro_x, gyro_y, gyro_z=sensor.gyro
dict_gx={'y0':gyro_y-avg_y, 'z0':gyro_z-avg_z}
dict_ax={'x0':accel_x-avg_accelx, 'z0':accel_z-avg_accelz}
time.sleep(T)
accel_x, accel_y, accel_z = sensor.accel
gyro_x, gyro_y, gyro_z=sensor.gyro
dict_gx={'y1':gyro_y-avg_y, 'z1':gyro_z-avg_z}
dict_ax={'x1':accel_x-avg_accelx, 'z1':accel_z-avg_accelz}
time.sleep(T)

# use lowpass filter 
dict_gy={'y0':0,'z0':0}
dict_ay={'x0':0,'z0':0}
dict_gy['y1']=dict_gx['y1']*(T*w)/(w*T+2)+dict_gx['y0']*(T*w)/(w*T+2)-dict_gy['y0']*(w*T-2)/(w*T+2)
dict_gy['z1']=dict_gx['z1']*(T*w)/(w*T+2)+dict_gx['z0']*(T*w)/(w*T+2)-dict_gy['z0']*(w*T-2)/(w*T+2)
dict_ay['x1']=dict_ax['x1']*(T*w)/(w*T+2)+dict_ax['x0']*(T*w)/(w*T+2)-dict_ay['x0']*(w*T-2)/(w*T+2)
dict_ay['z1']=dict_ax['z1']*(T*w)/(w*T+2)+dict_ax['z0']*(T*w)/(w*T+2)-dict_ay['z0']*(w*T-2)/(w*T+2)

if abs(dict_gy['y1'])<0.01:
    dict_gy['y1']=0
if abs(dict_ay['x1'])<0.01:
    dict_ay['x1']=0
if abs(dict_gy['z1'])<0.01:
    dict_gy['z1']=0
if abs(dict_ay['z1'])<0.01:
    dict_ay['z1']=0

# numerical integration to get yaw angle
dict_gv={'y0':0, 'z0':0}
dict_gv['z1']=(dict_gy['z1']+dict_gy['z0'])*T/2+dict_gv['z0']

# use complementary filter go get more accurate pitch angle
alpha = 0.95
dict_gv['y1'] = alpha*((dict_gy['y1']+dict_gy['y0'])*T/2+dict_gv['y0'])+(1-alpha)*math.atan2(dict_ay['z1'],dict_ay['x1'])*180/(2*math.pi)



while True:
    try:
        dict_gx['y0']=dict_gx['y1']
        dict_gx['z0']=dict_gx['z1']
        dict_gy['y0']=dict_gy['y1']
        dict_gy['z0']=dict_gy['z1']
        dict_gv['y0']=dict_gv['y1']
        dict_gv['z0']=dict_gv['z1']

        dict_ax['x0']=dict_ax['x1']
        dict_ax['z0']=dict_ax['z1']
        dict_ay['x0']=dict_ay['x1']
        dict_ay['z0']=dict_ay['z1']
    
        gyro_x, gyro_y, gyro_z = sensor.gyro
        accel_x, accel_y, accel_z = sensor.accel
        dict_gx['y1']=gyro_y-avg_y
        dict_gx['z1']=gyro_z-avg_z
        dict_ax['x1']=accel_x-avg_accelx
        dict_ax['z1']=accel_z-avg_accelz

        if abs(dict_gx['y1'])<0.01:
            dict_gx['y1']=0
        if abs(dict_gx['z1'])<0.01:
            dict_gx['z1']=0
        if abs(dict_ax['x1'])<0.01:
            dict_ax['x1']=0
        if abs(dict_ax['z1'])<0.01:
            dict_ax['z1']=0
    
        dict_gy['y1']=dict_gx['y1']*(T*w)/(w*T+2)+dict_gx['y0']*(T*w)/(w*T+2)-dict_gy['y0']*(w*T-2)/(w*T+2)
        dict_gy['z1']=dict_gx['z1']*(T*w)/(w*T+2)+dict_gx['z0']*(T*w)/(w*T+2)-dict_gy['z0']*(w*T-2)/(w*T+2)
        dict_ay['x1']=dict_ax['x1']*(T*w)/(w*T+2)+dict_ax['x0']*(T*w)/(w*T+2)-dict_ay['x0']*(w*T-2)/(w*T+2)
        dict_ay['z1']=dict_ax['z1']*(T*w)/(w*T+2)+dict_ax['z0']*(T*w)/(w*T+2)-dict_ay['z0']*(w*T-2)/(w*T+2)

        if abs(dict_gy['y1'])<0.01:
            dict_gy['y1']=0
        if abs(dict_gy['z1'])<0.01:
            dict_gy['z1']=0
        if abs(dict_ay['x1'])<0.01:
            dict_ay['x1']=0
        if abs(dict_ay['z1'])<0.01:
            dict_ay['z1']=0
    
        dict_gv['z1']=(dict_gy['z1']+dict_gy['z0'])*T/2+dict_gv['z0']
        dict_gv['y1'] = alpha*((dict_gy['y1']+dict_gy['y0'])*T/2+dict_gv['y0'])+(1-alpha)*math.atan2(dict_ay['z1'],dict_ay['x1'])*180/(2*math.pi)

        print('x axis accel-before filter: ', dict_ax['x1'], 'x axis accel-after filter: ', dict_ay['x1'],)
        print('z axis accel-before filter: ', dict_ax['z1'], 'z axis accel-after filter: ', dict_ay['z1'],)
        print('y axis gyro-before filter: ', dict_gx['y1'], 'y axis gyro-after filter: ', dict_gy['y1'],)
        print('z axis gyro-before filter: ', dict_gx['z1'], 'z axis gyro-after filter: ', dict_gy['z1'],)
        print('y axis angle: ', dict_gv['y1'], 'z axis angle: ', dict_gv['z1'],)      
