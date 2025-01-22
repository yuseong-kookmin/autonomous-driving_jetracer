from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Bool
import cv2
import numpy as np

is_stopline = False
is_stop = False

# 이미지 콜백 함수
def image_callback(msg):
    try:
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        process_videos(image)
      
        cv2.imshow("USB Camera Image", image)
        cv2.waitKey(1)  # 1밀리초 동안 대기

    except Exception as e:
        print(e)


def detect_stop_line(frame):
    # 대회 중에 ROI 범위 조정
    img_roi = frame[300:400, 100:600]
    
    img_gray = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)
    retval, img_binary = cv2.threshold(img_gray, 200, 255, cv2.THRESH_BINARY)

    white_count = cv2.countNonZero(img_binary)
    
    cv2.imshow('binary',img_binary)

    # ROI 범위에 따라 적당히 조정
    is_stopline = white_count > 15000

    return is_stopline


def process_videos(image):
    
        global is_stop
        
        is_stopline = detect_stop_line(image)

        print(f"is_stopline: {is_stopline},", end = ' ')

        is_stop = False
        
        if is_stopline:
            is_stop = True
        else:
            is_stop = False

        print(f"is_stop: {is_stop}")
            
        talker(is_stop)

        key = cv2.waitKey(20)   # 키보드 입력 대기 (영상 속도 조절)
         
       
def listener():
    rospy.init_node('usb_cam_subscriber', anonymous=True)
    
    # 이미지 토픽 구독자 설정
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    
    rospy.spin()
    
def talker(is_stop):
    
    pub = rospy.Publisher('is_stop', Bool,queue_size = 10)
        # is_stop 토픽으로 메시지 발행
    pub.publish(is_stop)
        

if __name__ == '__main__':
    listener()
