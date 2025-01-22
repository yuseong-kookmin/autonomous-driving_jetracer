#!/usr/bin/env python
# -*- coding: utf-8 -*-


from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Int32
import cv2
import numpy as np

is_stopline = False
is_stop = 0
is_green = False
m_s = 0
# 이미지 콜백 함수
def image_callback(msg):
    global m_s
    m_s = rospy.get_param("/m_s")
    if(m_s == 1 or m_s ==3 or m_s == 4):
	    try:
		# ROS 이미지 메시지를 OpenCV 이미지로 변환
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(msg, "bgr8")
		
		process_stopline(image)
	      
		
		cv2.waitKey(1)  # 1밀리초 동안 대기

	    except Exception as e:
		print(e)

def image_callback2(msg):
    global m_s
    m_s = rospy.get_param("/m_s")
    if(m_s == 1 or m_s == 3 or m_s==4):
	    try:
		# ROS 이미지 메시지를 OpenCV 이미지로 변환
		bridge2 = CvBridge()
		image2 = bridge2.imgmsg_to_cv2(msg, "bgr8")
		
		process_traffic(image2)
	      
		#cv2.imshow("traffic_frame",image2)
		cv2.waitKey(1)  # 1밀리초 동안 대기

	    except Exception as e:
		print(e)
        
     
     
def detect_traffic_light_color(frame):    
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 대회 중에 HSV 범위 조정
    green_on_lower = np.array([70, 100, 200])
    green_on_upper = np.array([100, 255, 255])
    
    green_off_lower = np.array([70, 100, 50])
    green_off_upper = np.array([100, 255, 100])
    
    green_on_mask = cv2.inRange(hsv_frame, green_on_lower, green_on_upper)
    green_off_mask = cv2.inRange(hsv_frame, green_off_lower, green_off_upper)
    
    green_on_count = cv2.countNonZero(green_on_mask)
    green_off_count = cv2.countNonZero(green_off_mask)
    
    is_green = green_on_count > 70

    return is_green


def detect_stop_line(frame):
    # 대회 중에 ROI 범위 조정
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    retval, img_binary = cv2.threshold(img_gray, 200, 255, cv2.THRESH_BINARY)

    white_count = cv2.countNonZero(img_binary)

    # ROI 범위에 따라 적당히 조정
    is_stopline = white_count > 55000

    return is_stopline


def process_traffic(image):
    
        global is_green
        image = image[200:400,100:]
        
        #cv2.imshow('traffic',image)
        key = cv2.waitKey(1)
        is_green = detect_traffic_light_color(image)

        #print("is_green:",is_green)
# 키보드 입력 대기 (영상 속도 조절)
def process_stopline(image):
    
        global is_stopline
        image = image[570:,]
        
        #cv2.imshow('stopline',image)
        key = cv2.waitKey(1)
        is_stopline = detect_stop_line(image)
	dec_stop()

        #print("is_stopline",is_stopline)

           # 키보드 입력 대기 (영상 속도 조절)
def dec_stop():
	
	if (is_stopline ==True and is_green == False):
		talker(1)
	elif(is_stopline == True and is_green == True):
		talker(2)
	else:
		talker(0)
	if(m_s == 5 and is_stopline == True):
		talker(1)
	
       
def listener():
    rospy.init_node('usb_cam_subscriber', anonymous=True)
    
    # 이미지 토픽 구독자 설정
    rospy.Subscriber("/usb_cam1/image_raw", Image, image_callback)
    rospy.Subscriber("/usb_cam2/image_raw", Image, image_callback2)
    
    rospy.spin()
    
def talker(is_stop):
    
    pub = rospy.Publisher('is_stop', Int32,queue_size = 10)
        # is_stop 토픽으로 메시지 발행
    pub.publish(is_stop)
        

if __name__ == '__main__':
    listener()
