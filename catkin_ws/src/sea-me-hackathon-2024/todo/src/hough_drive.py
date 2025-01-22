#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, math
import rospy, rospkg, time
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from math import *
from sensor_msgs.msg import Joy
import signal
import sys
import os
import random
import time


image = np.empty(shape=[0])
image2 = np.empty(shape=[0])
bridge = CvBridge()
bridge2 = CvBridge() # OpenCV 함수를 사용하기 위한 브릿지 
img_ready = False
img_ready2 = False # 카메라 토픽이 도착했는지의 여부 표시 


pub = None
Width = 1280
Height = 720
Offset = 340
Gap = 40
buttons = []

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 60    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 1280,720    # 카메라 이미지 가로x세로 크기

def img_callback(data):
    global image, img_ready
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    img_ready = True

def img_callback2(data2):
    global image2, img_ready2
    image2 = bridge2.imgmsg_to_cv2(data2, "bgr8")
    img_ready2 = True


def joy_callback(data):
    # Process the incoming Joy message data here
	global buttons 	
	buttons = list(data.buttons)


    

global lower_white
lower_white = np.array([150,150,150])
global upper_white
upper_white = np.array([255,255,255])
global lower_white2
lower_white2 = np.array([130,130,130])
global upper_white2
upper_white2 = np.array([255,255,255])

#=================================

#=================================
def color_filter(img):

    mask_white = cv2.inRange(img,lower_white,upper_white)
    white_image = cv2.bitwise_and(img,img,mask = mask_white)
    return white_image
def color_filter2(img):

    mask_white = cv2.inRange(img,lower_white2,upper_white2)
    white_image = cv2.bitwise_and(img,img,mask = mask_white)
    return white_image

def bird_eye_view(img,width,height):
	src = np.float32([[0,0],
                  [width,0],
                  [0,160],
                  [width,160]])

	dst = np.float32([[-100,0],
                  [width+90,0],
                  [160,height],
                  [460,height]])    
         
	M = cv2.getPerspectiveTransform(src,dst)
	M_inv = cv2.getPerspectiveTransform(dst,src)
	img_warped = cv2.warpPerspective(img,M,(width,height)) 
	return img_warped

def bird_eye_view2(img,width,height):
	src = np.float32([[0,0],
                  [width,0],
                  [0,160],
                  [width,160]])

	dst = np.float32([[60,0],
                  [width-60,0],
                  [250,height],
                  [390,height]])    
         
	M = cv2.getPerspectiveTransform(src,dst)
	M_inv = cv2.getPerspectiveTransform(dst,src)
	img_warped = cv2.warpPerspective(img,M,(width,height))  
	return img_warped

def region_of_interest(img):
	height = 480
	width = 640
	mask = np.zeros((height,width),dtype="uint8")

	pts = np.array([[0,0],[500,0],[500,480],[0,480]])  
	

	mask= cv2.fillPoly(mask,[pts],(255,255,255),cv2.LINE_AA)


	img_masked = cv2.bitwise_and(img,mask)
	return img_masked


def steering_Angle(R):
	if R == 0:
		return 0
	if R != 0:
		angle = np.arctan(0.34/R) 
		#if angle*180/np.pi < 0.03:  #Angle값이 매우 작은 경우 직진상태로 판단하여 0을 return 하도록 하였습니다.
		#	return 0
		#else:		
		return angle*180/np.pi # arctan로 계산한 값이 radian값이기 때문에 degree로 변환하여 return하였습니다.


def Radius(rx, ry):
	a=0
	b=0
	c=0
	R=0
	h=0
	w=0  #변수들을 초기화하는 과정입니다.
	if (rx[-3] - rx[11] != 0): 
		a = (ry[-3] - ry[11]) / (rx[-3] - rx[11])
		b = -1
		c = ry[11] - rx[11] * (ry[-3] - ry[11]) / (rx[-3] - rx[11])
		h = abs(a * np.mean(rx) + b * np.mean(ry) + c) / math.sqrt(pow(a, 2) + pow(b, 2))
		w = math.sqrt(pow((ry[-3] - ry[11]), 2) + pow((rx[-3] - rx[11]), 2))
             #rx,ry 는 각 window 조사창 내에 속해있는 흰색 픽셀들의 픽셀좌표의 평균을 담아놓은 리스트입니다.
	     #rx[-1]은 제일 위에있는 window를, rx[3]은 아래에서 4번째에 있는 window를 의미합니다.
	     #rx[0]대신 rx[3]을 이용한 이유는 시뮬레이터상의 카메라 높이가 낮아 차량에 가까운 차선이 인식이 불안정하였기 때문입니다.

	if h != 0:
		R = h / 2 + pow(w, 2) / h * 8
	
	return R*0.85/450
	#220 , 390

def mouse_callback(event,x,y,flags,param):
	if event == cv2.EVENT_LBUTTONDOWN:
		print("x=", x,"y=",y)
def sliding_window(img_masked,left_prob,right_prob):
    
    nwindows = 17
    window_height = 20 

    
    
 
    margin = 25
    minpix= 5
    
    out_img = np.dstack((img_masked, img_masked, img_masked)) * 255 

    histogram = np.sum(img_masked[img_masked.shape[0] // 2:, :], axis=0) 

    midpoint = 320
    leftx_current = np.argmax(histogram[:250])   
    rightx_current = np.argmax(histogram[midpoint+80:]) + midpoint+80

    #if(abs(leftx_current - x_temp) >30):
	#	leftx_current = x_temp
		
    #if(abs(rightx_current-y_temp)>30):
	#	rightx_current= y_temp
		
		
    y_temp = rightx_current    
    x_temp = leftx_current 
    nz = img_masked.nonzero()

    left_lane_inds = []
    right_lane_inds = []
    left_lane_inds2 = []
    right_lane_inds2 = []

    lx, ly, rx, ry = [], [], [], []
    lx2, ly2, rx2, ry2 = [], [], [], [] 
 

    

    for window in range(nwindows):

        win_yl = img_masked.shape[0] - (window + 1) * window_height 
        win_yh = img_masked.shape[0] - window * window_height

        win_xll = leftx_current - margin 
        win_xlh = leftx_current + margin 
        win_xrl = rightx_current - margin 
        win_xrh = rightx_current + margin


        good_left_inds = ((nz[0] >= win_yl) & (nz[0] < win_yh) & (nz[1] >= win_xll) & (nz[1] < win_xlh)).nonzero()[0]
        good_right_inds = ((nz[0] >= win_yl) & (nz[0] < win_yh) & (nz[1] >= win_xrl) & (nz[1] < win_xrh)).nonzero()[0] 

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds) 

        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nz[1][good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nz[1][good_right_inds])) 

        lx.append(leftx_current) 
        ly.append((win_yl + win_yh) / 2)

        rx.append(rightx_current)
        ry.append((win_yl + win_yh) / 2) 

        cv2.rectangle(out_img, (win_xll, win_yl), (win_xlh, win_yh), (0, 255, 0), 2)
        cv2.rectangle(out_img, (win_xrl, win_yl), (win_xrh, win_yh), (0, 255, 0), 2) 

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    

    out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
    out_img[nz[0][right_lane_inds], nz[1][right_lane_inds]] = [0, 0, 255] 



    

    return out_img ,lx,ly,rx,ry,good_left_inds,good_right_inds 


def hough(img):
	img_blur = cv2.GaussianBlur(img,(5,5),0)
	img_edge = cv2.Canny(img_blur,50,300)
	line = cv2.HoughLinesP(img_edge,1,np.pi/180,50,None,50,5)
	if line is not None:
		for i in range(0,len(line)):
			l = line[i][0]
			#cv2.line(img_edge,(l[0],l[1]),(l[2],l[3]),(0,0,255),3,cv2.LINE_AA)
	return img_edge
	


def start():
    rospy.init_node('h_drive')
    pub_steering = rospy.Publisher('/Steering', Float32, queue_size=1)
    pub_throttle = rospy.Publisher('/Throttle', Float32, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam1/image_raw/",Image,img_callback)
    #image_sub2 = rospy.Subscriber("/usb_cam2/image_raw/",Image,img_callback2)
    rospy.Subscriber("/joy", Joy, joy_callback)
    
    # Keep python from exiting until this node is stopped
    
    #lx,ly,rx,ry = [220,0,0,0,0,130] , [] , [390,0,0,0,0,130] , []   
    max_angle = 0
    left_lane_inds , right_lane_inds = [], []
    
    global image, image2, img_ready, motor
    time_flag = False
    avg_time = 0.0

    

    print ("----- Xycar self driving -----")

    left_prob,right_prob = 0,0
    lpos=rpos =0
    r_tmp = 570 
    ecnt= 0
    bcnt=0 
    tmp_angle =0
    f_yaw =0
    yaw_cnt = 0
    steering = -0.34
    throttle = 0
    cv2.namedWindow("warped")
    #cv2.namedWindow("warped2")

    

    while not rospy.is_shutdown():
		
		if not img_ready:
			continue
		#print("1")
		
		s_time = time.time()    
		#img1 = image.copy() 
		#img2 = image2.copy()
		#height,width = 720,1280
			
		
		#cv2.imshow("warped",img1)
		#cv2.imshow("warped2",img2)
		
		
		
		#print(time.time()-s_time)
		cv2.waitKey(1)

		if(len(buttons)==16):
			if(buttons[3] == 1):
				throttle = 0.0
				pub_throttle.publish(throttle)
				pub_steering.publish(steering)
			elif(buttons[1] == 1):
				throttle = 0.2
				pub_throttle.publish(throttle)
				pub_steering.publish(steering)
			elif(buttons[0] == 1):
				throttle = -0.2			
				pub_throttle.publish(throttle)
				pub_steering.publish(steering)

		
		
	
	

		
		
    	
#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
#=============================================
if __name__ == '__main__':
    start()

