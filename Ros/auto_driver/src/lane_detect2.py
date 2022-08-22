#!/usr/bin/env python
#-*- coding:utf-8 -*-

import time
import rospy
import numpy as np
import threading
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from bluetooth_bridge.msg import LaneMsg

# GLOBAL VARIABLES
lane_vel = Twist()
angularScale = 6 # 180/30
servodata = 0

traffic_light_data = 0 # 0-red, 1-green, 2-yellow
ped_cross_data = 0
speed_state_data = 0 # 0-normal, 1-unlimited, 2-limited, 3-limited_min
scan_filter = []

actual_motor_speed_data = 0
actual_vehicle_mode_data = 0
actual_vehicle_direction_data = 0
supersonic_distance_data = 0
thetaX_data = 0

bridge = 0
robot = 0

def thread_job():
    rospy.spin()

def lanecallback(msg):
    global lane_vel, angularScale
    lane_vel = msg
    _servoCmdMsg = msg.angular.z * angularScale + 90
    global servodata
    servodata = min(max(0, _servoCmdMsg), 180)
    servodata = 100 - servodata * 100 / 180
    # rospy.loginfo(rospy.get_caller_id() + 'lane_vel.angular.z = %f', lane_vel.angular.z)

def lightcallback(msg):
    global traffic_light_data
    traffic_light_data = msg.data
    # rospy.loginfo(rospy.get_caller_id() + 'traffic_light_data is %s', traffic_light_data)

def pedcrosscallback(msg):
    global ped_cross_data
    ped_cross_data = msg.data
    # rospy.loginfo(rospy.get_caller_id() + 'ped_cross_data is %s', ped_cross_data)

def speedstatecallback(msg):
    global speed_state_data
    speed_state_data = msg.data
    # rospy.loginfo(rospy.get_caller_id() + 'speed_state_data is %s', speed_state_data)

def scancallback(msg):
    global scan_filter
    scan_filter = msg.ranges

def speedcallback(msg):
    global actual_motor_speed_data
    actual_motor_speed_data = msg.data
    # rospy.loginfo(rospy.get_caller_id() + 'actual_motor_speed_data is %s', actual_motor_speed_data)

def modecallback(msg):
    global actual_vehicle_mode_data
    actual_vehicle_mode_data = msg.data
    # rospy.loginfo(rospy.get_caller_id() + 'actual_vehicle_mode_data is %s', actual_vehicle_mode_data)

def directioncallback(msg):
    global actual_vehicle_direction_data
    actual_vehicle_direction_data = msg.data
    # rospy.loginfo(rospy.get_caller_id() + 'actual_vehicle_direction_data is %s', actual_vehicle_direction_data)

def distancecallback(msg):
    global supersonic_distance_data
    supersonic_distance_data = msg.data
    # rospy.loginfo(rospy.get_caller_id() + 'supersonic_distance_data is %s', supersonic_distance_data)

def thetaXcallback(msg):
    global thetaX_data
    thetaX_data = msg.data
    # rospy.loginfo(rospy.get_caller_id() + 'thetaX_data = msg.data is %s', thetaX_data)

def robot_avoidance():
    global scan_filter

    while len(scan_filter) == 0:
        pass

    left = min(scan_filter[180*4:220*4])
    right = min(scan_filter[160*4:180*4])

    # try:
        # print('min distance of left', left)
        # print('min distance of right', right)
        # print('angle of left', 180 + 0.25 * scan_filter[180*4:260*4].index(left))
        # print('angle of left', 160 + 0.25 * scan_filter[160*4:180*4].index(right))
    # except:
        # pass

    if left < 0.7 or right < 0.7:
        print('robot stop!')
        return 1
    else:
        return 0

def parking():
    global scan_filter
    while len(scan_filter) == 0:
        pass

    left_back = min(scan_filter[340*4:])
    right_back = min(scan_filter[:20*4])
    back = min(left_back, right_back)
    if back < 1:
        return 1
    else:
        return 0

def bridge_slowdown():
    global thetaX_data
    print("thetaX_data",thetaX_data)
    if thetaX_data > 1000: #上坡
        return 1
    elif thetaX_data >= 0 and thetaX_data <= 500: #平路
        return 2
    else:
        return 0


def kinematicCtrl():
    global servodata
    global traffic_light_data
    global ped_cross_data
    global speed_state_data
    global scan_filter
    global actual_motor_speed_data
    global actual_vehicle_mode_data
    global actual_vehicle_direction_data
    global supersonic_distance_data
    global thetaX_data
    global bridge
    global robot

    lane_pub = rospy.Publisher("/lane_detect", LaneMsg, queue_size=10)
    # pub1 = rospy.Publisher('/bluetooth/received/manul', Int32, queue_size=10)
    # pub2 = rospy.Publisher('/auto_driver/send/direction', Int32, queue_size=10)
    # pub3 = rospy.Publisher('/auto_driver/send/speed', Int32, queue_size=10)
    # pub4 = rospy.Publisher('/auto_driver/send/gear', Int32, queue_size=10)

    manul = 0 # 0-Automatic, 1-Manual
    direction = 50 # 0-left-50-right-100
    speed = 0 # 0-100
    gear = 2 # 1-Drive, 2-Neutral, 3-Park, 4-Reverse

    servodata_list = []

    red_stop_seen = 0
    yellow_back_seen = 0
    park_forward_cnt = 0
    park_back_cnt = 0
    park = 0

    pedestrain = 0
    ped_cnt = 0

    unlimit_seen = 0
    limit_seen = 0
    limit_cnt = 0

    bridge_count = 0

    rospy.init_node('kinematicCtrl', anonymous=True)


    add_thread = threading.Thread(target=thread_job)
    add_thread.start()

    rate = rospy.Rate(10) # 10Hz
    rospy.Subscriber('/lane_vel', Twist, lanecallback)
    rospy.Subscriber('/traffic_light', Int32, lightcallback)
    rospy.Subscriber('/ped_cross', Int32, pedcrosscallback)
    rospy.Subscriber('/speed_state', Int32, speedstatecallback)
    rospy.Subscriber('/scan', LaserScan, scancallback)

    rospy.Subscriber('/vcu/ActualMotorSpeed', Int32, speedcallback)
    rospy.Subscriber('/vcu/ActualVehicleMode', Int32, modecallback)
    rospy.Subscriber('/vcu/ActualVehicleDirection', Int32, directioncallback)
    rospy.Subscriber('/vcu/SupersonicDistance', Int32, distancecallback)
    rospy.Subscriber('/vcu/thetaX', Int32, thetaXcallback)

    rospy.loginfo(rospy.is_shutdown())

    n = 1 # 1Hz
    servodata_list = n * [servodata]
    while not rospy.is_shutdown():
        servodata_list[0:n-1] = servodata_list[1:n]
        servodata_list[n-1] = servodata

        servoSum = 0
        for i in servodata_list:
            servoSum += i
        
        servodata_mean = servoSum / n
        direction = servodata_mean

        #
        #
        # if traffic_light_data == 0 and red_stop_seen == 0:
        #     speed = 0
        #     gear = 2
        # elif traffic_light_data == 1:
        #     red_stop_seen = 1
        #     speed = 28
        #     gear = 1
        # elif traffic_light_data == 2 and yellow_back_seen == 0:
        #     speed = 28
        #     gear = 1
        #     park_forward_cnt = park_forward_cnt + 1
        #     if park_forward_cnt == 50:
        #         yellow_back_seen = 1
        # else:
        #     pass
        #
        # print('traffic_light_data', traffic_light_data)
        # print('yellow_back_seen', yellow_back_seen)
        # print('park_forward_cnt', park_forward_cnt)
        #
        # if pedestrain == 0 and ped_cross_data == 1:
        #     speed = 0
        #     gear = 3
        #     ped_cnt = ped_cnt + 1
        #     if ped_cnt == 20:
        #         pedestrain = 1
        # else:
        #     pass
        #
        # if speed_state_data == 0:
        #     speed = 28
        #     gear = 1
        # elif speed_state_data == 1:
        #     pass
        # elif limit_seen == 0 and speed_state_data == 2:
        #     limit_seen = 1
        #     speed = 10
        #     gear = 1
        # elif speed_state_data == 3:
        #     speed = 28
        #     gear = 1
        # else:
        #     pass
        # if limit_seen == 1:
        #     speed = 10
        #     gear = 1
        #     limit_cnt = limit_cnt + 1
        #     if limit_cnt == 300:
        #         limit_seen = 2
        # print('limit_seen:', limit_seen)
        #
        #
        # if yellow_back_seen == 2:
        #     break
        #
        # if yellow_back_seen == 1:
        #     speed = 25
        #     direction = 50
        #     gear = 4
        #     if parking() == 1:
        #         gear = 3
        #         speed = 0
        #         park = 1
        #         yellow_back_seen = 2
        #
        #
        # if bridge_slowdown() == 1 and bridge == 0:
        #     speed = 28
        #     bridge = 1 #bridge = 1在桥上（上坡）
        #     gear = 1
        #
        # if bridge == 1:
        #     speed = 28
        #     gear = 1
        #     bridge_count += 1
        #     if bridge_count >= 240:
        #         bridge = 2 #bridge=2 在桥顶（准备下坡）
        #         speed = 5
        #         gear = 1
        #
        # if bridge == 2 and bridge_slowdown() == 2:
        #     speed = 0
        #     gear = 3
        #     bridge_count += 1
        #     if bridge_count >= 260:
        #         speed = 28
        #         gear = 1
        #         bridge = 0
        # print("bridge",bridge)
        # print("bridge_slowdown",bridge_slowdown())
        #
        # if red_stop_seen == 0:
        #     speed = 0
        #     gear = 2
        #
        # robot = robot_avoidance()
        # if robot == 1:
        #     speed = 0
        #     gear = 3
        # else:
        #     pass

        direction = max(direction, 15)
        direction = min(direction, 70)
        lane_pub.publish(LaneMsg(bias=0.0, gear=direction))
        # print('traffic_light', traffic_light_data)
        # print('red_stop_seen', red_stop_seen)
        #
        # print('manul', manul)
        # print('direction:', direction)
        # print('speed', speed)
        # print('gear', gear)


        # pub1.publish(manul)
        # pub2.publish(direction)
        # pub3.publish(speed)
        # pub4.publish(gear)
        rate.sleep()
    
if __name__ == '__main__':
    kinematicCtrl()





