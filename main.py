import RPi.GPIO as GPIO
import time
import math
import board
import busio
import time
import adafruit_lsm9ds1
import pygame
from pygame.locals import *   # for event MOUSE variables
import os

os.putenv('SDL_VIDEODRIVER', 'fbcon')   # Display on piTFT
os.putenv('SDL_FBDEV', '/dev/fb1')
os.putenv('SDL_MOUSEDRV', 'TSLIB') # Track mouse clicks on piTFT
os.putenv('SDL_MOUSEDEV', '/dev/input/touchscreen')

#pygame initilization
pygame.init()
pygame.mouse.set_visible(False)
WHITE = 255, 255, 255
BLACK = 0,0,0
screen = pygame.display.set_mode((320, 240))

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

# set GPIO pins
trig=24
echo=21

pwma=4
ain2=6
ain1=5

bin1=13
bin2=19
pwmb=26

standby=17

encoder = 16

#initilization
dist_meas = 0.00
km_h = 0
elapse = 0
pulse = 0

def init_GPIO():
    GPIO.setwarnings(False)
    GPIO.setup(encoder,GPIO.IN,GPIO.PUD_UP)

# wheel encoder functions
def calculate_elapse(channel):
    global pulse,start_timer,elapse
    pulse+=1
    elapse = time.time() - start_timer
    start_timer = time.time()

def calculate_speed(r_cm):
    global pulse,elapse,rpm,dist_meas,km_h,km_s
    if elapse !=0:
        rpm=1/(elapse*20) *60
        circ_cm = (2*math.pi*r_cm)
        dist_km = circ_cm/100000
        km_s = dist_km/(elapse*20)
        km_h = km_s *3600
        dist_meas = (dist_km*pulse)*1000
        return km_s

# first 2 left turns function 
def left():
    global dict_v, temp, close, add_d, adjust, count
    thres=90
    if abs(dict_v['z1']-temp)<thres:
        add_d=0
        t_left() 
    else:     
        forward()
        add_d=1
        adjust=1
        
# Forward
def forward():
    pwml.ChangeDutyCycle(60)
    pwmr.ChangeDutyCycle(60)
    GPIO.output(ain1, 1)
    GPIO.output(ain2, 0)
    GPIO.output(bin1, 1)
    GPIO.output(bin2, 0)

# General Turn left
def t_left():
    pwml.ChangeDutyCycle(0)
    pwmr.ChangeDutyCycle(40)
    GPIO.output(ain1, 1)
    GPIO.output(ain2, 1)
    GPIO.output(bin1, 1)
    GPIO.output(bin2, 0)

#Turn right
def t_right():
    pwml.ChangeDutyCycle(40)
    pwmr.ChangeDutyCycle(0)
    GPIO.output(ain1, 1)
    GPIO.output(ain2, 0)
    GPIO.output(bin1, 1)
    GPIO.output(bin2, 1)

def stop():
    pwml.ChangeDutyCycle(0)
    pwmr.ChangeDutyCycle(0)
    GPIO.output(ain1, 1)
    GPIO.output(ain2, 1)
    GPIO.output(bin1, 1)
    GPIO.output(bin2, 1)

# define encoder interrupt
def init_interrupt():
    GPIO.add_event_detect(encoder,GPIO.FALLING,callback = calculate_elapse,bouncetime = 10)

#define two buttons' callback functions
def GPIO27_callback(channel):
    global codeRun
    codeRun=2
    
def GPIO23_callback(channel):
    global codeRun
    codeRun=1

# add button detection  
GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)
GPIO.add_event_detect(23, GPIO.FALLING, callback=GPIO23_callback, bouncetime=300)

# initial setup
GPIO.setup(ain1,GPIO.OUT)
GPIO.setup(ain2,GPIO.OUT)
GPIO.setup(pwma,GPIO.OUT)
GPIO.setup(bin1,GPIO.OUT)
GPIO.setup(bin2,GPIO.OUT)
GPIO.setup(pwmb,GPIO.OUT)
GPIO.setup(standby,GPIO.OUT)
GPIO.output(standby,GPIO.HIGH)
   
GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)

GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pwml=GPIO.PWM(pwma, 1000)
pwmr=GPIO.PWM(pwmb, 1000)

pwml.start(0)
pwmr.start(0)

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

GPIO.output(trig, False)
time.sleep(2)


# IMU calibration
my_font= pygame.font.Font(None, 20)
my_buttons= { 'Calibrating...':(160,120) }
screen.fill(BLACK)               # Erase the Work space

for my_text, text_pos in my_buttons.items():
    text_surface= my_font.render(my_text, True, WHITE)
    rect= text_surface.get_rect(center=text_pos)
    screen.blit(text_surface, rect)
pygame.display.flip()

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

my_buttons= { 'Calibratiion Finished':(160,60),'Press Button #23 to start':(160,180) }
screen.fill(BLACK)               # Erase the Work space

for my_text, text_pos in my_buttons.items():
    text_surface= my_font.render(my_text, True, WHITE)
    rect= text_surface.get_rect(center=text_pos)
    screen.blit(text_surface, rect)
pygame.display.flip()


codeRun=0

# press button #23 to start
while codeRun==0:
    pass

my_buttons= { 'Start!':(160,120) }
screen.fill(BLACK)               # Erase the Work space

for my_text, text_pos in my_buttons.items():
    text_surface= my_font.render(my_text, True, WHITE)
    rect= text_surface.get_rect(center=text_pos)
    screen.blit(text_surface, rect)

forward()

start_timer=time.time()
start=time.time()
d=0

init_GPIO()
init_interrupt()

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


i=0 # encoder count
j=0 # ranger count
close=0 # whether it is too close
slope=0 # whether on slope
count=0 # turn count
size=[] # dimension of area
flag=0 # angle tolerance
turn2=0 # flag of second 90 degree turn in 's' route u-turn
add_d=1 # whether should add distance, no add when turning
adjust=0 # whether turning should be adjusted
straight=0 # indicate straight moving between two 90 degree turn in 'S' route u-turn
mapping=0 # start to draw map

al=0 # initial altitude

# how to divid the area length and width 
uy = 4
ux = 4

#block unit length in piTFT
unit_xmap=320/ux
unit_ymap=240/uy

# block ID list
col_id = []

# Maximum Height/altitude
mh = 0

while codeRun==1:
    #read ranger
    if j%5==0:
        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)

        while GPIO.input(echo)==0:
            pulse_start=time.time()

        while GPIO.input(echo)==1:
            pulse_end=time.time()

        pulse_duration=pulse_end-pulse_start

        distance=pulse_duration*17150
        distance=round(distance,2)
        print('distance: ', distance)
        j=0
    
    j+=1

    #update imu       
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

    # detect walls and avoid collision
    if distance < 13 and close==0:
        print('too close! turn right!')
        close=1
        temp=dict_v['z1']
        count+=1
        if count-2==ux:
            stop()
            codeRun=2
            break 
        if count==1:
            size.append(round(d,2))
        elif count==2:
            size.append(round(d-size[0],2))
            print('size: ', size)
            l_x=size[1]
            w_y=size[0]
            # get block dimension in cm
            unit = [w_y*100/uy, l_x*100/ux]
            # record start 's' route moving distance 
            ud = d

    if close==1:
        # first 2 90 degree turn when moving along the edge
        if (count==1 or count==2) and adjust==0 :
            left()
        elif (count==1 or count==2) and adjust>=1:
            a_angle=90
            # forward a little bit to let integration result catch up with real angle
            adjust+=1
            # if still not 90 degree, continue turning, otherwise forward
            if (dict_v['z1']-(temp-a_angle)<0 and adjust>=20):
                forward()
                adjust=0
                close=0
            elif dict_v['z1']-(temp-a_angle)>0 and adjust >= 20:
                t_left()

        # adjustment of second 90 degree turn in u-turn in 's' route
        elif count>2 and adjust>=1 and turn2==2:
            '''
            adjust+=1
            if count%2==1:
                tt=38
            else:
                tt=30
            '''
            adjust += 1
            if (dict_v['z1']-(temp-tt)<0 and adjust>=20):
                forward()
                adjust=0
                close=0
                turn2=0
            elif dict_v['z1']-(temp-tt)>0 and adjust>=20:
                if count%2==1:
                    t_left()
                else:
                    t_right()

        # u-turn in 's' route
        else:
            # left u turn
            if count%2==1:
                # first 90 degree turn
                if abs(dict_v['z1']-temp)<90 and turn2==0:
                    add_d=0
                    t_left()
                # second 90 degree turn
                elif abs(dict_v['z1']-temp)<90 and turn2==1 and (d-temp_d)*100>=unit[1]:
                    add_d=0
                    t_left()
                    straight=0
                # forward in between
                else:
                    add_d=1
                    forward()
                    if turn2==0 and straight==0:
                        temp_d=d
                        straight=1
                        temp=dict_v['z1']
                        turn2=1
                    elif turn2==1 and straight==0:
                        turn2=2
                        adjust=1
                        
            # right u turn
            else:
                if abs(dict_v['z1']-temp)<90 and turn2==0:
                    add_d=0
                    t_right()
                elif abs(dict_v['z1']-temp)<90 and turn2==1 and (d-temp_d)*100 >= unit[1]:
                    add_d=0
                    t_right()
                    straight=0
                else:
                    add_d=1
                    forward()
                    if turn2==0 and straight==0:
                        temp_d=d
                        temp=dict_v['z1']
                        straight=1
                        turn2=1
                    elif turn2==1 and straight==0:
                        turn2=2
                        adjust=1
       
    i+=1   
    if i%10==0:
        calculate_speed(3.2)
        current=time.time()
        if add_d==1:
            d+=(current-start)*km_s*1000
        start=time.time()
        i=0
        print('pulse',pulse,'speed',km_h,'dis',d)

    # detect whether moving on a slope
    if dict_v['y1']-dict_v['y0']>=0.03 and slope==0:
        d_begin=d
        angle_begin=dict_v['y0']
        slope=1
        print('begin: ', d_begin, angle_begin)

    # if on a slope...
    if slope==1:
        # whether reach peak of the slope
        if dict_v['y1']-dict_v['y0']<=0:
            flag+=1 
            if flag >=3:
                angle_finish=dict_v['y0']
                d_finish=d
                # calculate the altitude
                al=abs((d_finish-d_begin)*math.sin((angle_finish-angle_begin)*math.pi/180))
                slope=0
                flag=0
                print('finish: ', d_finish, angle_finish)
                print('altitude: ', al)
                if (abs(al)>mh):
                    mh = al
                
    # get current robot position's block id and piTFT coordinate
    if mapping==1:
        # if on a slope, get x direction movement
        if slope==1:
            diff = d_begin-ud + (d-d_begin)*math.cos((dict_v['y1']-angle_begin)*math.pi/180)
        else:
            diff=d-ud
        a=diff*100%(size[0]*100+unit[1])
        b=int(diff*100/(size[0]*100+unit[1]))
        c=math.ceil(a/unit[1])
        uid=uy*b+c
        cor_x=int((uid-1)/uy)*unit_xmap
        if uid != 0:
            if int((uid-1)/4)%2==0:
                cor_y=int((uid-1)%uy)*unit_ymap
            else:
                cor_y=int(((b+1)*uy-uid)%uy)*unit_ymap
            print(uid,a,b,c,cor_x,cor_y)

            if len(col_id)==0:
                col_id.append([cor_x, cor_y, al, BLACK, uid])

            # if moving to a new block, add to the list  
            elif col_id[-1][4]!=uid:
                col_id.append([cor_x, cor_y, al, BLACK, uid])
                al=0
                if mh!=0:
                    for m in range(len(col_id)-1):
                        # change block color if containing slope
                        if col_id[m][2]!=0:
                            color = abs(int((255*col_id[m][2]/mh)))
                            col_id[m][3]=[0,color,color]
                            print(col_id[m][3])

    # blit information when moving along edge
    if count < 2:
    
        my_buttons= { 'ranger:':(100,20), 'encoder:':(100,60), 'y angle:':(100,100), 'z angle:':(100,140), 'al:':(100,180),'size:':(100,220)}
        screen.fill(BLACK)               # Erase the Work space

        for my_text, text_pos in my_buttons.items():
            text_surface= my_font.render(my_text, True, WHITE)
            rect= text_surface.get_rect(center=text_pos)
            screen.blit(text_surface, rect)


        if count >=2:
            text_surface= my_font.render(str(size), True, WHITE)
            rect= text_surface.get_rect(center=(200,220))
            screen.blit(text_surface, rect)


        
        text_surface= my_font.render(str(distance), True, WHITE)
        rect= text_surface.get_rect(center=(200,20))
        screen.blit(text_surface, rect)
    
        text_surface= my_font.render(str(round(d,3)), True, WHITE)
        rect= text_surface.get_rect(center=(200,60))
        screen.blit(text_surface, rect)
    
        text_surface= my_font.render(str(round(dict_v['y1'],4)), True, WHITE)
        rect= text_surface.get_rect(center=(200,100))
        screen.blit(text_surface, rect)
    
        text_surface= my_font.render(str(round(dict_v['z1'],4)), True, WHITE)
        rect= text_surface.get_rect(center=(200,140))
        screen.blit(text_surface, rect)
    
        pygame.display.flip()

    # blit real time map when moving in 's' direction
    elif mapping==1:
        screen.fill(WHITE)
        for n in range(len(col_id)):
            pygame.draw.rect(screen, col_id[n][3], [col_id[n][0], col_id[n][1], unit_xmap, unit_ymap])

        pygame.display.flip()
        
    time.sleep(T)


# finish scanning the whole area
print(col_id)
stop()
pwml.ChangeDutyCycle(0)
pwmr.ChangeDutyCycle(0)

# show the complete map
while codeRun==2:
    for r in range(len(col_id)):
        pygame.draw.rect(screen, col_id[r][3], [col_id[r][0], col_id[r][1], unit_xmap, unit_ymap])

    text='Press button #23 for detailed information'
    text_surface= my_font.render(text, True, WHITE)
    rect= text_surface.get_rect(center=(160,20))
    screen.blit(text_surface, rect)
    pygame.display.flip()

# find max peak block coordinate
for h in range(len(col_id)):
    if col_id[h][2] == mh:
        maxc=[col_id[h][0], col_id[h][1]]
mh=round(mh*100,2)

# blit summarized information
while codeRun==1:
    screen.fill(WHITE)
    size_info='size: '+str(size)
    text_surface= my_font.render(size_info, True, BLACK)
    rect= text_surface.get_rect(center=(160,40))
    screen.blit(text_surface, rect)

    text='maximum altitude: '+str(mh)+' cm' 
    text_surface= my_font.render((text), True, BLACK)
    rect= text_surface.get_rect(center=(160,120))
    screen.blit(text_surface, rect)

    text='at coordinate '+str(maxc)
    text_surface= my_font.render((text), True, BLACK)
    rect= text_surface.get_rect(center=(160,150))
    screen.blit(text_surface, rect)

    text='Press Button #27 to quit'
    text_surface= my_font.render((text), True, BLACK)
    rect= text_surface.get_rect(center=(160,200))
    screen.blit(text_surface, rect)

    pygame.display.flip()

GPIO.cleanup()


