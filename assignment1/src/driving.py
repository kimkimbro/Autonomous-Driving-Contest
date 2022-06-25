#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
from email.quoprimime import header_decode
from tempfile import tempdir
import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================

image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge() 
motor = None  # 모터 토픽을 담을 변수

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================

CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480 # 카메라 이미지 가로x세로 크기

# warp 변환 이미지 크기
warp_img_w = 640
warp_img_h = 480


nwindows = 15 #sliding window 개수
margin = 50 # window 가로 크기 margin
minpix = 900 # window 안에 들어와야 하는 최소 pixel 갯수
lane_bin_th = 170 # binary 이진화를 진행할때 pixel threshold
speed=10
x =  0 # angle control 에서 eld 값을 구하기 위한 좌표 값 위치
lxd = 248 # 이미지 h - x -> look ahead distance 380 
l = 34 # 차량의 크기 30으로 설정
k = 0.0001 # 차량 
angle = 0
prev_angle=0

# warp 변환 전 pixel 좌표값 (4 points) 
warp_src  = np.array([
    [160, 320],  
    [20, 380],
    [480, 320],
    [620, 380]
], dtype=np.float32)

# warp 변환 후 매칭 될 pixel 좌표값 (4 points)
warp_dist = np.array([
    [0,0],
    [0,warp_img_h],
    [warp_img_w,0],
    [warp_img_w, warp_img_h]
], dtype=np.float32)

#=============================================

# warp 변환 함수
def warp_image(img, src, dst, size):
    M = cv2.getPerspectiveTransform(src, dst) # 투시변환 변환 행렬
    Minv = cv2.getPerspectiveTransform(dst, src) # 투시변환 역행렬
    warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR) # warp 변환 
    back = np.zeros((480,320,3),np.uint8) # 검은화면 생성

    # warp 변환 이미지 왼쪽 오른 쪽에 margin 값을 위한 검은색 배경 확장
    warp_img = cv2.hconcat([back,warp_img]) 
    warp_img = cv2.hconcat([warp_img,back])

    return warp_img, M, Minv

#이진화 함수
def filter_image(img):
    _, _, B = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2LAB)) # 횡단보도를 나타내는 노란색 pixel 값 filtering 을 위하여 lab 채널로 변환
    img[np.where(B >= 140)] = (0,0,0) # 특정 threshold 값을 주어 노란색 pixel 값 0으로 처리
    _, L, _ = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HLS)) # 차선을 나타내는 흰색 pixel 값 filtering을 위하여 HLS 채널로 변환
    _, lane = cv2.threshold(L, lane_bin_th, 255, cv2.THRESH_BINARY) # 특정 threshold 값을 주어 차선만 검출
    
    return lane

# sliding window 생성 및 경로 함수값 추출     
def warp_process_image(lane):
    global nwindows
    global margin
    global minpix
    global lane_bin_th
        
    histogram = np.sum(lane[lane.shape[0]-lane.shape[0]//nwindows:,:], axis=0) # 맨 아래 window y 값안에 있는 pixel 들의 sum 값을 구함
    midpoint = np.int(histogram.shape[0]/2) 
    if np.all(histogram[:midpoint] == 0):
        leftx_current = 375
    else:
        leftx_current = np.argmax(histogram[:midpoint]) # 왼쪽 영역의 hot pixel의 x 좌표값을 찾아 왼쪽 차선의 위치를 저장 
    if np.all(histogram[midpoint:] == 0):
        rightx_current = 905
    else:
        rightx_current = np.argmax(histogram[midpoint:]) + midpoint
         # 오른쪽 영역의 hot pixel의 x 좌표값을 찾아 오른쪽 차선의 위치를 저장

    window_height = np.int(lane.shape[0]/nwindows) # window 의 height 구함
    nz = lane.nonzero() # 이진화 영상에서 0값이 아닌 즉 차선인 부분의 좌표 값을 nz에 저장

    left_lane_inds = [] # 왼쪽 차선의 모든 pixel 값을 저장할 배열 선언
    right_lane_inds = [] # 오른쪽 차선의 모든 pixel 값을 저장할 배열 선언
    
    lx, ly, rx, ry = [], [], [], [] # sliding window 의 중심의 좌표값을 저장할 배열 선언

    out_img = np.dstack((lane, lane, lane))*255 ## sliding window 표시를 위해 rgb 채널로 확장

    # window를 돌며 점들의 집합을 구하여 poly 형태로 왼쪽, 오른쪽 차선 유도
    for window in range(nwindows):

    # sliding window 좌표값을 유도해내는 과정 , sliding window 내의 차선 부분 좌표값을 평균 내어 sliding window 중심값 변경
        for i in range (2):
            # window 의 y 좌표값 
            win_yl = lane.shape[0] - (window+1)*window_height
            win_yh = lane.shape[0] - window*window_height

            # window 의 x 좌표값 
            win_xll = leftx_current - margin
            win_xlh = leftx_current + margin
            win_xrl = rightx_current - margin
            win_xrh = rightx_current + margin

            # window 내의 pixel 값들 저장
            good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]
            good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]

            # window의 pixel 갯수에 따라 추출된 hot pixel값을 경우의 수로 나누어 계산
            if len(good_left_inds) > minpix and len(good_right_inds) > minpix : # 양쪽 다 추출
                leftx_current = np.int(np.mean(nz[1][good_left_inds]))
                rightx_current = np.int(np.mean(nz[1][good_right_inds]))
            elif len(good_left_inds) > minpix and len(good_right_inds) < minpix : # 왼쪽만 추출 
                leftx_current = np.int(np.mean(nz[1][good_left_inds])) 
                rightx_current = leftx_current + 505 # 왼쪽 차선 기준으로 도로 거리폭 만큼 떨어진 가상의 hot pixel 점 생성
            elif len(good_left_inds) < minpix and len(good_right_inds) > minpix : # 오른쪽만 추출
                rightx_current = np.int(np.mean(nz[1][good_right_inds]))
                leftx_current = rightx_current-505 # 오른쪽 차선 기준으로 도로 거리폭 만큼 떨어진 가상의 hot pixel 점 생성

        # sliding window 표시 왼쪽 파랑, 오른쪽 빨강
        cv2.rectangle(out_img,(win_xll,win_yl),(win_xlh,win_yh),(255,0,0), 2) 
        cv2.rectangle(out_img,(win_xrl,win_yl),(win_xrh,win_yh),(0,0,255), 2) 

        # sliding window 내의 차선 좌표값들 저장
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        # 왼쪽 오른쪽 hot pixel 값 , 즉 sliding window의 중심값 저장, 이 점들을 통해 차선을 2차 함수로 유도함
        lx.append(leftx_current)
        ly.append((win_yl + win_yh)/2)

        rx.append(rightx_current)
        ry.append((win_yl + win_yh)/2)

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # 구한 점들의 집합을 가지고 왼쪽 오른쪽 차선의 함수를 유도
    lfit = np.polyfit(np.array(ly),np.array(lx),2)
    rfit = np.polyfit(np.array(ry),np.array(rx),2)

    # 왼쪽 오른쪽 차선을 가지고 따라 가야할 중심 경로를 구하여 out_img에 표시하는 함수, 밑에서 설명예정
    draw_line(out_img,lfit,rfit)

    # 차선 점들의 집합 표시 왼쪽 파랑, 오른쪽 빨강
    out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
    out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]] = [0, 0, 255]

    cv2.imshow("viewer", out_img)
    
    return lfit, rfit

# img에 따라가야 할 경로 시각화
def draw_line(img, lfit, rfit):
    c_line = (lfit+rfit)/2 # 중심 경로 추출
    x = np.linspace(0,480,481) # 0~480 x값 생성
    y = c_line[0]*x**2 +c_line[1]*x + c_line[2] # x 값에 따른 y 값 생성

    # image에 표현하기 위하여 type 변경
    x= x.astype(np.int32)
    y= y.astype(np.int32)
    
    pts = np.array([np.transpose(np.vstack([y,x]))]) # 각 x,y의 점들의 집합을 point 형식 [x,y]형태의 배열로 변환
    cv2.polylines(img, pts, False, (255,255,0),3)

# 생성된 경로를 따라가기 위하여 pure pursuit 알고리즘 사용
def control(lfit, rfit):
    c_line = (lfit+rfit)/2
    global x, lxd, l, angle, k, prev_angle

    x = 0
    #Look Ahead distance는 Pure Pursuit Control의 main tuning 요소. 차량이 현재 위치에서 얼마나 멀리 있는 path를 봐야하는지 결정.
    #너무 가깝게 설정하면 overshoot이 심해져 불안정, 너무 멀게 설정하면 path를 따라가는 반응 속도가 느려짐.  tuning과정을 걸쳐 적절한 값으로 설정
    lxd=2.48*100 #Look ahead distance 설정, steering 입력 값 결정에 매우 중요한 값.
    l=0.34*100 #차량 길이
    #생성한 2차함수 곡선에서 픽셀의 위치가 y=480일 때 x값
    center = c_line[0]*480**2 +c_line[1]*480 +c_line[2] 
    eld = abs((c_line[0]*x**2 +c_line[1]*x + c_line[2])-640)*0.32 # 중심값인 640에서 얼만큼 함수가 떨어져 있는지 eld 계산
    #print("eld : ",eld)
    ld = (lxd**2+eld**2)**0.5 # 현재 위치에서 원하는 goal point까지의 직선 거리 ld 계산
    k = 2*eld/(ld*ld) # 1/차량의 반경 , 곡률

    #heading angle 값을 통해 추종하는 경로 함수에서 차량 위치의 접선의 기울기를 파악
    #윈도우 픽셀에서 y 값이 480이기 때문에 아래와 같이 계산, 접선의 기울기를 구하기 위해 함수를 y로 한 번 미분
    heading = (c_line[0]*2*480+c_line[1])*180/math.pi #슬라이딩 윈도우를 통해 구한 중앙선의 heading

    # 중심 640을 기준으로 가야 할 위치(goal point)에 따른 angle 값을 수식적으로 유도
    if c_line[0]*x**2 +c_line[1]*x +c_line[2] >center :  # 중심보다 오른쪽에 가야할 지점이 있는 경우 
        angle = atan(((2*l*eld)/(ld*ld)))
    else:  # 중심보다 왼쪽에 가야할 지점이 있는 경우 
        angle = -atan(((2*l*eld)/(ld*ld)))

    angle = angle*180/math.pi # rad to degree
    print("angle: ", angle)

    # 480(현재 차량의 위치)에서의 중심 차선의 위치와 이미지의 중앙값과의 error 값을 구하여 P제어, 이를 통해 차선의 중앙을 더욱 잘 tracking 할 수 있음      
    thr = (center)-640 #이미지 중앙값과 현재 경로의 480에서의 값의 차이
    P = 0.13 # P제어 상수 초기값

   # angle 제어 시 P제어 이용, 위에서 수식적으로 구한 angle을 바탕으로 추종해야 하는 angle 값에 따라 맞는 P 상수 값을 설정
   # 그에 따라 steering이 부드럽게 나타나는 효과를 얻을 수 있음
   # angle이 큰 경우는 경로를 빠르게 따라가야 하기 때문에 높은 P 상수 값을 줌
   # angle이 작은 경우는 speed를 올려야함. angle이 쉽게 변화하면 차량이 크게 흔들림. 이에 따라 steering의 변화량을 줄여주기 위해 낮은 P 상수 값을 줌
    if heading >40 or heading <-40 or angle>15 or angle <-15: #슬라이딩 윈도우 영상의 노이즈로 인해 수식적으로 계산한 angle 값이 정상범위를 벗어나는 경우
        #순간적으로 노이즈로 인해 angle 값이 급변하는 것을 방지하기 위해 직전의 angle값에 큰 가중치를 줌
        P=0.15
        angle = angle + P * thr # angle P 제어
        angle = (0.9*prev_angle + 0.1*angle) #바로 직전 angle값의 가중치를 많이 주어 steering이 튀는 것을 방지 및 현재 값도 적절히 반영
        prev_angle = angle # 전 angle값을 활용할 수 있도록 새로운 변수에 저장해둠
    elif angle >5 or angle <-5:
        P = 0.14
        angle = angle + P * thr # angle P 제어
        prev_angle = angle
    elif angle >1 or angle <-1:
        P = 0.13
        angle = angle + P * thr # angle P 제어
        prev_angle = angle       
    else: # 차량이 거의 직진에 가깝게 주행하는 경우, P 값을 매우 낮게 주어 차량 angle이 천천히 변화하도록 함
        P=0.1
        angle = angle + P * thr # angle P 제어
        prev_angle = angle

    return angle 

def control_speed(angle):
    global speed

    #추종하는 경로의 curve가 큰 구간에 진입 시 속도를 낮추고 curve가 낮은 곳에 진입 시 속도를 높이는 방법으로 속도 제어
    #차량의 angle값에 따라 적절한 속도를 P제어를 통해 구현, 적절한 속도를 tuning 과정을 거치며 최적의 speed 값을 찾음
    #주행 test를 거치며 구간 별로 angle 값을 출력해 본 후, 각 구간마다 적절한 speed 설정 및 P값 설정
    if angle > 15 or angle <-15: # 커브가 큰 구간에는 속도를 급격히 낮추는 경우가 발생하므로 큰 p값을 설정
        proper_velocity=20
        p_speed=0.05
    elif angle >10  or angle <-10:
        #proper_velocity=17
        proper_velocity=22
        p_speed=0.04     
    elif angle >5  or angle <-5:
        #proper_velocity=20
        proper_velocity=23
        p_speed=0.05        
    elif angle >1  or angle <-1: #커브기 낮은 곳에서는 속도를 빠르게 증가시키기 위해 P값을 높여 줌
        proper_velocity=25
        p_speed=0.06       
    else:
        proper_velocity=30
        p_speed=0.07

    thr = proper_velocity - speed # 속도 에러 값 계산
    speed = speed + p_speed * thr # 속도를 P 제어
    print("speed: ", speed)
    return speed

# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 변수에 옮겨 담음.
#=============================================

def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 모터 토픽을 발행하는 함수  
# 입력으로 받은 angle과 speed 값을 
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================

def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)


#=============================================
# 실질적인 메인 함수 
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함. 
#=============================================

def start():

    # 위에서 선언한 변수를 start() 안에서 사용하고자 함

    global motor, image

    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================

    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    print ("----- Xycar self driving -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue
     #=========================================
    # 메인 루프 
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서 
    # "이미지처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
    # 작업을 반복적으로 수행함.
    #=========================================

    while not rospy.is_shutdown():

        # 이미지처리를 위해 카메라 원본이미지를 img에 복사 저장
        img = image.copy()  
        # warp 변환 
        warp_img, M, Minv = warp_image(img, warp_src, warp_dist, (warp_img_w, warp_img_h))
        # 차선 추출 및 이진화
        lane = filter_image(warp_img)
        # sliding window 기법을 활용한 왼쪽 오른쪽 차선의 2차 함수 유도
        left_fit, right_fit = warp_process_image(lane)
        # 추출된 함수를 기반으로 angle P 제어
        angle = control(left_fit,right_fit)
        #ldf_set(angle)
        speed = control_speed(angle)

        # 디버깅을 위해 모니터에 이미지를 디스플레이
        pts  = np.array([
                    [160, 320],  
                    [20, 380],
                    [620, 380],
                    [480, 320]
                ], dtype=np.int32)
        cv2.fillConvexPoly(img,pts,(0,255,0))

        cv2.imshow("CAM View", img)
        cv2.waitKey(1)       

        # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
        drive(angle, speed)

#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
#=============================================
if __name__ == '__main__':
    start()