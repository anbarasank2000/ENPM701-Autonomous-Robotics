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
trig = 16
echo = 18


def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT)
    gpio.setup(33, gpio.OUT)
    gpio.setup(35, gpio.OUT)
    gpio.setup(37, gpio.OUT)
    gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)
    gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP)
def gameover():
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)
    gpio.cleanup()
def move_right(pin_left, pin_right, ang):
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
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    count = 0
    while True:
        if (ser.in_waiting>0):
            count+=1
            line = ser.readline()
            if count==12:
                line = line.rstrip().lstrip()
                #print(line)
                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")
                des_ang_x = (float(line[3:6]))
            if count>20:
                line = line.rstrip().lstrip()
                #print(line)
                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")
                cur_ang_x = (float(line[3:6]))
                if int(gpio.input(12)) != int(button_BR):
                    button_BR = int(gpio.input(12))
                    counter_BR +=1
                # print(counter_BR)
                if int(gpio.input(7)) != int(button_FL):
                    button_FL = int(gpio.input(7))
                    counter_FL +=1
                    # print(counter_FL)
                error = (counter_BR - counter_FL)*2
                pwm_L = 60 + error
                pwm_R = 60 - error
                pwm1.ChangeDutyCycle(pwm_L)
                pwm2.ChangeDutyCycle(pwm_R)
                print(cur_ang_x,des_ang_x)
                if cur_ang_x>= des_ang_x+(ang) or cur_ang_x<= des_ang_x-(ang):
                    error = (counter_BR - counter_FL)*2
                    pwm_L = 40 + error
                    pwm_R = 40 - error
                    pwm1.ChangeDutyCycle(pwm_L)
                    pwm2.ChangeDutyCycle(pwm_R)
                if cur_ang_x>= des_ang_x+ang or cur_ang_x<= des_ang_x-ang:
                    pwm1.stop()
                    pwm2.stop()
                    #go front with imu,dist feed
                    gameover()
                    #print(cur_ang_x,des_ang_x)
                    print("Goodbye")
                    break

f = open('hw4data.txt','a')
# initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640,480))
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('arrow.avi', fourcc, 3, (640, 480))
# allow the camera to warmup
time.sleep(0.1)
cor = []
# keep looping
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
    # grab the current frame
    image = frame.array
    # show the frame to our screen
    # cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF
    a = image.shape
    img = np.zeros((a[0],a[1],1), dtype=np.uint8)
    # cv2.imshow("black", img)
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    lower_range = np.array([133, 80, 140]) #lower range of HSV red home
    upper_range = np.array([255, 255, 255]) #lower range of HSV
    #SHOWING THE MASKED HSV 
    hsv_mask = cv2.inRange(image_hsv, lower_range, upper_range)
    hsv_mask = cv2.GaussianBlur(hsv_mask, (5,5), 
                       cv2.BORDER_DEFAULT)
    #cv2.imshow("contour", hsv_mask)
    contour, _ = cv2.findContours(hsv_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contour = contour[0]
    (x,y),rad = cv2.minEnclosingCircle(contour) 
      
    center = (int(x),int(y)) 
    radius = int(rad)
    print(center)
    cv2.circle(image, center, radius, (0,0,255), 2)
    offset = (image.shape[1])/2-center[0]
    if offset>25:
        #print("left by: ", offset)
        angle = 0.061*(offset)
        #move_right(33,37,angle)
    elif (-1*offset)>25:
        #print("right by: ", (-1*offset))
        angle = 0.061*(-1*offset)
        #move_right(31,35,angle)
        
        
    cv2.imshow("circle", image)
    # plt.imshow(img),plt.show()
    cv2.waitKey(1)
    
    out.write(image)
    rawCapture.truncate(0)
    if key == ord("q"):
        
        break