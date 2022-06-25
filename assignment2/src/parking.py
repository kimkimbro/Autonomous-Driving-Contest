#! /usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import rospy, math
import cv2, time, rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
arData = {"DX":0.0, "DY":0.0, "DZ":0.0,            # DXYZ는 좌표 위치
          "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}  # AXYZW는 각도 값 
roll, pitch, yaw = 0, 0, 0
motor_msg = xycar_motor()

#=============================================
# 콜백함수 - ar_pose_marker 토픽을 처리하는 콜백함수
# ar_pose_marker 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 AR 정보를 꺼내 arData 변수에 옮겨 담음.
#=============================================
def callback(msg):
    global arData

    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x   
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w

#=========================================
# ROS 노드를 생성하고 초기화 함.
# AR Tag 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
#=========================================
rospy.init_node('ar_drive')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1 )

#=========================================
# 메인 루프 
# 끊임없이 루프를 돌면서 
# "AR정보 변환처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
# 작업을 반복적으로 수행함.
#=========================================
ward = True

while not rospy.is_shutdown():

    # 쿼터니언 형식의 데이터를 오일러 형식의 데이터로 변환
    (roll,pitch,yaw)=euler_from_quaternion((arData["AX"],arData["AY"],arData["AZ"], arData["AW"]))
	
    # 라디안 형식의 데이터를 호도법(각) 형식의 데이터로 변환
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    
    # Row 100, Column 500 크기의 배열(이미지) 준비
    img = np.zeros((100, 500, 3))

    # 4개의 직선 그리기
    img = cv2.line(img,(25,65),(475,65),(0,0,255),2)
    img = cv2.line(img,(25,40),(25,90),(0,0,255),3)
    img = cv2.line(img,(250,40),(250,90),(0,0,255),3)
    img = cv2.line(img,(475,40),(475,90),(0,0,255),3)

    # DX 값을 그림에 표시하기 위한 좌표값 계산
    point = int(arData["DX"]) + 250

    if point > 475:
        point = 475

    elif point < 25 : 
        point = 25	

    # DX값에 해당하는 위치에 동그라미 그리기 
    img = cv2.circle(img,(point,65),15,(0,255,0),-1)  
  
    # DX값과 DY값을 이용해서 거리값 distance 구하기
    distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DY"],2))
    
    # 그림 위에 distance 관련된 정보를 그려넣기
    cv2.putText(img, str(int(distance))+" pixel", (350,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))

    # DX값 DY값 Yaw값 구하기
    dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"])) \
                +" Yaw:"+ str(round(yaw,1)) 


    # 그림 위에 DX값 DY값 Yaw값 관련된 정보를 그려넣기
    cv2.putText(img, dx_dy_yaw, (20,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))

    # 만들어진 그림(이미지)을 모니터에 디스플레이 하기
    cv2.imshow('AR Tag Position', img)
    cv2.waitKey(1)



    ############### 추가한 코드 ###############
    
    ## 전진과 후진을 결정하는 ward라는 bool 변수를 65번째 줄에 선언하였음.
    ## ward가 true일 때, 전진.(거리가 가까운데 DX가 0이 아닐 때 제외)  /   false일 때, 후진.
    
    
    yaw_limit = 0.5         ##    최종 yaw의 threshold
    weight_param = 8.4      ##    145번째 줄에 angle에 적용할 yaw의 weight
    slow_distance = 216    ##  주차벽과의 거리가 가까워졌다고 생각한 거리. 섬세한 주차를 필요로 하는 거리이다.
    
    ### 전진 실시!
    if(ward):
        if(distance <=73 and distance >= 55):  # 거리가 가까울 때(정차해야 할 때)
            if(abs(yaw) > yaw_limit):  # 틀어져 있다면(yaw값이 yaw_limit 초과일 때)  
                ward = False  # 후진 실시
                if(speed == 0):   # 다음 실행을 위해 만약 speed가 0인 정지 상태임을 인지했다면
                    ward = True;   #  다음 초기에 전진 실행
            else: # yaw값이 2 이하일 때
                if(abs(arData["DX"]) < 1):  # 제대로 주차되어 있다면 정지
                    angle = 0
                    speed = 0
                else: # DX 값이 0이 아니라면 DX를 0으로 맞춰주기 위한 후진
                    ward = False;   # 후진 실시 
                    if(speed == 0):   # 다음 실행을 위해 만약 speed가 0인 정지 상태임을 인지했다면
                        ward = True;   #  다음 초기에 전진 실행
        else:  # 주차칸에 다가가는 경우
            if((arData["DY"]!=0)):  # 처음 실행할 때 speed를 일시적으로 주는 경우를 제외한 일반적인 경우
                angle = (distance/2)*math.atan(float(arData["DX"])/arData["DY"]) - weight_param * yaw
                print(angle)
                ##  DX/DY의 atan 값의 각을 angle값에 반영하였고, 거리에 따른 angle 값에 가중치(distance/2)를 부여하여 주차칸에 가까워지기 전에 DX가 0에 가까울 수 있도록 함
                ##  DX값뿐만 아니라 yaw값 또한 yaw_limit 미만으로 맞추어야 올바른 주차라고 생각했기 때문에 yaw 값을 반영하였고, 이에 적정 weight를 곱했다. 
                if(distance<=slow_distance):  # 주차칸에 차가 가까워졌을 때
                    speed = abs(7/72*distance + 20)   #   거리에 따라 속도를 점점 줄이기 위한 일차함수를 설정함.
                else:   #   주차칸에서 멀리 있을 때
                    heading = yaw + math.atan(float(arData["DX"])/arData["DY"])*180/math.pi
                    ##   heading은 DX/DY의 각 + yaw 값으로 주차벽과의 각도를 의미한다.
                    if(abs(heading) >= 15):   # 주차벽과의 각도의 절댓값이 15도 이상일 때, 속도를 제한
                        speed =  -0.2*abs(heading) + 47    # 주차벽과의 각도의 크기에 따른 속도를 설정함. 각이 클수록 속도를 줄임.
                    else:  #  거의 직선 구간일 때
                        speed = 50  # 속도 50
            else:   # 처음 실행할 때, 즉 DY값이 0으로 되어있어서 움직이지 않는 문제로 인해 speed를 50으로 주어 시작과 동시에 이동.
                angle = 0
                speed = 50
                ward = True
        

    ### 후진 실시!
    if(ward == False):
        heading = yaw + math.atan(float(arData["DX"])/arData["DY"])*180/math.pi
        ## heading은 DX/DY의 각 + yaw 값으로 주차벽과의 각도를 의미한다.
        if(heading <= 0.5 and distance > slow_distance):  # 후진하고 있는데 heading 값이 0.5 이하이고 거리가 slow_distance 초과이면
        ## 후진을 한번 실행할 때, 주차벽으로부터 거리가 일정 거리가 될 때 까지는 후진을 실시함. 그 이후에 주차벽과의 각이 작다면 직진으로 들어가서 주차 진행.
            ward = True   #  전진 수행
        else:       
        ## 현재 과정은 heading을 0으로 맞추는 과정이며, DX와 yaw 값을 모두 0에 가깝게 맞추는 것과 같다.
        ## DX와 yaw 값을이 모두 0에 가깝게 맞춰지면, 다시 전진!
            if(heading<0):  # 주차벽과의 각도가 0보다 작을 때(왼쪽으로 틀어져 있을 경우), 들어온 angle값에서 조금씩 더해주면서 부드러운 후진을 진행
                angle += 0.05
                speed = -10
            elif(heading>0): # 주차벽과의 각도가 0보다 클 때(오른쪽으로 틀어져 있을 경우), 들어온 angle값에서 조금씩 빼주면서 부드러운 후진을 진행
                angle -= 0.05
                speed= -10

            
    ##################### 작성 코드 끝 #######################    
            
  

    # 핸들 조향각 angle값 설정하기
    # angle = 50
		
    # 차량의 속도 speed값 설정하기
    # speed = 5	
     
    # 조향각값과 속도값을 넣어 모터 토픽을 발행하기
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor_pub.publish(motor_msg)


# while 루프가 끝나면 열린 윈도우 모두 닫고 깔끔하게 종료하기
cv2.destroyAllWindows()





