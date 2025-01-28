# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import imutils
import numpy as np
import RPi.GPIO as gpio
import numpy as np
import time
import serial
import threading

ser = serial.Serial('/dev/ttyUSB0', 9600)
count = 0
center = 0
current_angle = 0
trig = 16
echo = 18
pix = [371, 266, 354, 345, 339, 315, 337, 338, 337, 336, 335, 333, 329, 324, 319, 315, 310, 305, 299, 282, 269, 261, 252, 246, 242, 238, 233,
       231, 227, 225, 223, 221, 219, 217]

leng = [9, 9.5, 10, 10.5, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70,
        75, 80, 85, 90, 95, 100]

def interpol(cy):
    for i, val in enumerate(pix):  
        if val < cy or i == len(pix)-1:
            break
    if i == 0:  
        dist = leng[0]
    else: 
        dist = leng[i] + (cy - pix[i]) * (leng[i-1] - leng[i]) / (pix[i-1] - pix[i])
        dist = round(dist, 3)
    return dist

def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT)
    gpio.setup(33, gpio.OUT)
    gpio.setup(35, gpio.OUT)
    gpio.setup(37, gpio.OUT)
    gpio.setup(36, gpio.OUT)
    gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)
    gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP)
    
def gameover():
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)
    gpio.cleanup()

def distance():
    gpio.setmode(gpio.BOARD)
    gpio.setup(trig, gpio.OUT)
    gpio.setup(echo, gpio.IN)
    gpio.output(trig, False)
    time.sleep(0.01)
    gpio.output(trig, True)
    time.sleep(0.00001)
    gpio.output(trig, False)
    while gpio.input(echo) == 0:
        pulse_start = time.time()

    while gpio.input(echo) == 1:
        pulse_end = time.time()

    pulse_dur = pulse_end - pulse_start
    distance = pulse_dur * 17150
    distance = round(distance, 2)
    gpio.cleanup()
    return distance

def average_dist(num_readings=10):
    dist_list = []
    for _ in range(num_readings):
        dist_list.append(distance())
        time.sleep(0.1)  
    average_distance = sum(dist_list) / len(dist_list)
    print(average_distance)
    return average_distance

def turn(g_ang, g_direc):
    init()
    if g_direc=="C":
        pin_left = 31
        pin_right = 35
    else:
        pin_left = 33
        pin_right = 37
    counter_FL = np.uint64(0)
    counter_BR = np.uint64(0)
    button_FL = int(0)
    button_BR = int(0)
    pwm1 = gpio.PWM(pin_left,50)
    pwm2 = gpio.PWM(pin_right,50)
    val = 0
    pwm1.start(val)
    pwm2.start(val)
    time.sleep(0.1)
    count = 0
    while True:
        if (ser.in_waiting>0):
            count+=1
            line = ser.readline()
            if count==12:
                line = line.rstrip().lstrip()
                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")
                des_ang_x = (float(line))%359
                if g_direc=="C":
                    goal = (360+(des_ang_x+g_ang))%360
                else:
                    goal = (360+(des_ang_x-g_ang))%360
            if count>20:
                line = line.rstrip().lstrip()
                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")
                cur_ang_x = (float(line))%359
                if int(gpio.input(12)) != int(button_BR):
                    button_BR = int(gpio.input(12))
                    counter_BR +=1
                if int(gpio.input(7)) != int(button_FL):
                    button_FL = int(gpio.input(7))
                    counter_FL +=1
                error = (counter_BR - counter_FL)*2
                pwm_L = 60 + error
                pwm_R = 60 - error
                pwm_R = max(min(90, pwm_R), 0)
                pwm_L = max(min(90, pwm_L), 0)
                pwm1.ChangeDutyCycle(pwm_L)
                pwm2.ChangeDutyCycle(pwm_R)
                print(cur_ang_x,des_ang_x)
#                 if cur_ang_x>= des_ang_x+(0.7*ang) or cur_ang_x<= des_ang_x-(0.7*ang):
#                     error = (counter_BR - counter_FL)*2
#                     pwm_L = 40 + error
#                     pwm_R = 40 - error
#                     pwm_R = max(min(90, pwm_R), 0)
#                     pwm_L = max(min(90, pwm_L), 0)
#                     pwm1.ChangeDutyCycle(pwm_L)
#                     pwm2.ChangeDutyCycle(pwm_R)
                ang,direc = good_theta(cur_ang_x, goal)
                if ang<4.05:
                    if direc != g_direc:
#                 if cur_ang_x>= des_ang_x+(ang) or cur_ang_x<= des_ang_x-(ang):
                        pwm1.stop()
                        pwm2.stop()
                        gameover()
                        print("Goodbye")
                        break
    return cur_ang_x

def move(pin_left, pin_right, dist):
    enc_ticks = dist*4701*0.94
    init()
    counter_FL = np.uint64(0)
    counter_BR = np.uint64(0)
    button_FL = int(0)
    button_BR = int(0)

    pwm1 = gpio.PWM(pin_left,50)
    pwm2 = gpio.PWM(pin_right,50)
    val = 0
    pwm1.start(val)
    pwm2.start(val)
    time.sleep(0.1)
    count = 0
    while True:
        if (ser.in_waiting>0):
            count+=1
            line = ser.readline()
            if count==12:
                line = line.rstrip().lstrip()
                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")
                des_ang_x = (float(line))%359
            if count>20:
                line = line.rstrip().lstrip()
                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")
                cur_ang_x = float(line)
                error = (des_ang_x - cur_ang_x)
                if (-1*error)>300:
                    error+=360
                error = error*17 
                pwm_L = 40 + error
                pwm_R = 40 - error
                pwm_R = max(min(90, pwm_R), 0)
                pwm_L = max(min(90, pwm_L), 0)
                pwm1.ChangeDutyCycle(pwm_L)
                pwm2.ChangeDutyCycle(pwm_R)
        if int(gpio.input(12)) != int(button_BR):
            button_BR = int(gpio.input(12))
            counter_BR +=1
        if int(gpio.input(7)) != int(button_FL):
            button_FL = int(gpio.input(7))
            counter_FL +=1
        
        if counter_BR>= enc_ticks and counter_FL>= enc_ticks:
            pwm1.stop()
            pwm2.stop()
            gameover()
            print("Goodbye")
            break
    return counter_BR,counter_FL

def move_gripper(pwm_val):
    init()
    pwm = gpio.PWM(36, 50)
    pwm.start(5)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(pwm_val)
    time.sleep(0.5)
    pwm.stop()
    gpio.cleanup()

def localization(c_r, c_l, curr_angle, present_coord):
    past_x, past_y = present_coord
    print("ang for loc",curr_angle)
    distance_front_left = c_l / (4701*0.94)
    distance_back_right = c_r / (4701*0.94)
    distance = (distance_front_left + distance_back_right) / 2.0
    print(distance, "m travelled with ang", curr_angle, "with coord", past_x, past_y)
    delta_x = round((distance * (np.sin(np.radians(curr_angle)))) + past_x, 2)
    delta_y = round((distance * (np.cos(np.radians(curr_angle)))) + past_y, 2)
    print("curr pose",delta_x, delta_y)
    return (delta_x, delta_y)
def good_theta(cur_ang, to_ang):
    if cur_ang == 180:  
        cur_ang += 0.01
    if (cur_ang > 180 and to_ang > 180) or (cur_ang < 180 and to_ang < 180):  
        dist = abs(cur_ang - to_ang)
        direc = "C" if to_ang > cur_ang else "CC"
    else:  
        dist_pi = abs(180 - cur_ang) + abs(180 - to_ang)
        dist_zero = 360 - dist_pi
        if dist_zero < dist_pi:  
            dist = dist_zero
            direc = "C" if cur_ang > 180 else "CC"
        else:  
            dist = dist_pi
            direc = "C" if cur_ang < 180 else "CC"
    print(dist, direc)
    return dist, direc
def col_det(block_col):
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    if block_col=="r":
        lower_range = np.array([133, 175, 140])
        upper_range = np.array([255, 255, 255]) #lower range of HSV
    elif block_col=="g":
        lower_range = np.array([40, 80, 60]) #lower range of HSV green home
        upper_range = np.array([80, 255, 255]) #lower range of HSV
    elif block_col=="b"
        lower_range = np.array([100, 140, 70]) #lower range of HSV blue home
        upper_range = np.array([140, 255, 255]) #lower range of HSV
    #SHOWING THE MASKED HSV 
    hsv_mask = cv2.inRange(image_hsv, lower_range, upper_range)
    hsv_mask = cv2.GaussianBlur(hsv_mask, (5,5), 
                       cv2.BORDER_DEFAULT)
    contour, _ = cv2.findContours(hsv_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    try:
        contour = contour[0]
    except IndexError:
        return 0,0
    (x,y),rad = cv2.minEnclosingCircle(contour) 
    center = (int(x),int(y)) 
    radius = int(rad)
    interpol(center[1])
    cv2.circle(image, center, radius, (0,0,255), 2)
    offset = (image.shape[1])/2-center[0]
    cv2.imshow("circle", image)
    return offset, center[1]

def corner_localization(curr_angle):
    ang,direc = good_theta(curr_angle, 270)
    curr_angle = turn(ang,direc)
    x = average_dist()
    ang,direc = good_theta(curr_angle, 90)
    curr_angle = turn(ang,direc)
    y = average_dist()
    return x,y

def frame_cap():
    if rawCapture.array is not None:
        rawCapture.truncate(0)
    camera.capture(rawCapture, format="bgr", use_video_port=False)
    image = rawCapture.array
    return image

def to_block(block_col):
    global current_angle
    offset, pix_height = col_det(block_col)
    if offset == 0:
        return 0,0,0
    if offset>30:
        angle = 0.061*(offset)*1.25*0.9
        print("left by: ", angle)
        current_angle = turn(angle, "CC")
    elif (-1*offset)>30:
        angle = 0.061*(-1*offset)*1.25*0.9
        print("right by: ", angle)
        current_angle = turn(angle, "C")
    print("curr_ang", current_angle)
    return offset, pix_height, current_angle

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640,480))
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('arrow.avi', fourcc, 3, (640, 480))

# allow the camera to warmup
time.sleep(0.1)
cor = []
live_coord = []
present_coord = (2.0,1.0)
goal_coord = (1.0,1.0)
blocks_order = ["r", "g", "b"]

# keep looping
for block in blocks_order:
    block_col = block
    while True:
        live_coord.append(present_coord)
        image = frame_cap()
        offset, pix_height, curr_angle = to_block(block_col)
        if offset == 0:  # if no block go to search points
            print("no block")
            turn(30, "CC")
            continue
        if offset<=30:
            #move straight
            dist = interpol(pix_height)*(2/3)
            print("interpol", dist, "cm")
            dist = (dist -15)/100.0
            print("moving", dist, "m")
            if dist<0.10:
                #open gripper
                move_gripper(7.5)
                _, _, curr_angle = to_block()
                c_r, c_l = move(31,37, 0.2)
                move_gripper(3.5)
                print("block picked")
                present_coord = localization(c_r, c_l, curr_angle, present_coord)
                construct_dist = ((present_coord[0]-goal_coord[0])**2 + (present_coord[1]-goal_coord[1])**2 )**0.5
                ang = (360 - np.arctan2(goal_coord[1] - present_coord[1], goal_coord[0] - present_coord[0]) * 360/(2*np.pi)) % 360
                ang+=90
                print("before", ang)
                ang,direc = good_theta(curr_angle, ang)
                print("arctan theta", ang, direc)
                curr_angle = turn(ang,direc)
                c_r, c_l = move(31,37, construct_dist)
                present_coord = localization(c_r, c_l, curr_angle, present_coord)
                move_gripper(7.5)
                break
            
            c_r, c_l= move(31,37, dist)
            present_coord = localization(c_r, c_l, curr_angle, present_coord)

        key = cv2.waitKey(1) & 0xFF
        cv2.waitKey(1)
        out.write(image)
        rawCapture.truncate(0)
        if key == ord("q"):
            break
        
    c_r, c_l = move(33,35, 0.2) #move reverse
    new_x, new_y = corner_localization(curr_angle)
    present_coord = (new_x, new_y)

    
    
