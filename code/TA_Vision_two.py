#!/usr/bin/env python
# -*- coding:utf8 -*-

import rospy
import sys
import math
import time
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from math import copysign
import json
import numpy as np
import time
import cv2
import paho.mqtt.client as mqtt
from threading import *
import paho.mqtt.client as mqtt
from collections import OrderedDict
from std_msgs.msg import Int32, Float32, Float32MultiArray

class Vision_Mqtt():
    def __init__(self):
        self.mqtt_host = "192.168.3.6"
        self.mqtt_port = 50001
        self.mqttClient = mqtt.Client()
        self.TD_Vision_Type = "Unknow"
        self.TD_Vision_ID = 0
        #self.start_mqtt()

    # 连接MQTT服务器
    def on_mqtt_connect(self):
        self.mqttClient.connect(self.mqtt_host, self.mqtt_port, 60)

    # subscribe 消息
    def on_subscribe(self):
        self.mqttClient.subscribe("/HG_DEV/Vision_MSG", 2)
        self.mqttClient.on_message = self.on_message_come  # 消息到来处理函数

    def start_mqtt(self):
        self.on_mqtt_connect()
        self.on_subscribe()
        self.mqttClient.loop_start()

    # publish 消息
    def on_publish(self, payload, topic="/HG_DEV/Vision_REQ", qos=2):
        print("pub msg: %s to %s" % (payload, topic))
        self.mqttClient.publish(topic, payload, qos)

    # 消息处理函数
    def on_message_come(self, lient, userdata, msg):
        print("from: " + msg.topic + " " + ":" + msg.payload)
        sub_msg = json.loads(msg.payload.encode('utf-8'))
        if sub_msg["name"] == "ZK" and sub_msg["dir"] == "TD":
            self.TD_Vision_Type = sub_msg["action"]
            self.TD_Vision_ID = sub_msg["target_id"]

class TA_Vision():
    def __init__(self):
        rospy.init_node("ta_visvion_node")
        self.host_ip = rospy.get_param("~host_ip", "192.168.3.6")
        self.host_port = rospy.get_param("~host_port", "50001")

        self.x1_dis = rospy.get_param("~x1_dis", 5.0) # x方向补偿,mm
        self.y1_dis = rospy.get_param("~y1_dis", 10.0) # y方向补偿,mm
        self.z1_dis = rospy.get_param("~z1_dis", -35.0) # z方向补偿,mm

        self.x2_dis = rospy.get_param("~x2_dis", 5.0) # x方向补偿,mm
        self.y2_dis = rospy.get_param("~y2_dis", 10.0) # y方向补偿,mm
        self.z2_dis = rospy.get_param("~z2_dis", -35.0) # z方向补偿,mm

        # 视觉识别模式,0:AGV上没有码,盲放; 1:AGV上有码,识别放置
        self.vision_mode = rospy.get_param("~vision_mode", "0")
        
        self.ta_current_x = 0.0
        self.ta_current_y = 0.0
        self.ta_current_z = 0.0

        #   mqtt消息
        self.ta_vision_req_msg = OrderedDict()
        self.ta_vision_req_msg["name"] = "TD"
        self.ta_vision_req_msg["dir"] = "ZK"
        self.ta_vision_req_msg["action"] = "SL"
        self.ta_vision_req_msg["target_pose"] = [0.0, 0.0, 0.0]
        self.ta_vision_req_json = json.dumps(self.ta_vision_req_msg)

        self.ar_pose_array = np.zeros((9, 7))
        self.push_state = 0

        # mqtt连接
        self.TA_Vision_Mqtt = Vision_Mqtt()
        self.TA_Vision_Mqtt.mqtt_host = self.host_ip
        self.TA_Vision_Mqtt.mqtt_port = self.host_port
        self.TA_Vision_Mqtt.start_mqtt()

        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.get_ar_pose)  #   订阅ar码识别话题
        rospy.Subscriber("get_push_pose", Int32, self.get_push_state)   #   订阅放置位姿话题

        self.ta_target_pose = rospy.Publisher('/target_pose', Float32MultiArray, queue_size=5)  #   发布目标位姿话题
        self.ta_target_pose1 = rospy.Publisher('/target_pose1', Float32MultiArray, queue_size=5)  #   发布目标位姿话题

        self.push_target_pose = rospy.Publisher('/push_target_pose', Float32MultiArray, queue_size=5)   #   发布放置目标位姿话题

        self.ta_control_pos = rospy.Subscriber("/Pall_CURR_POS", Float32MultiArray, self.get_ta_pose)   #   订阅当前塔吊位姿

    def get_push_state(self, data):
        if data.data == 1:
            self.push_state = 1

    #   
    def get_ta_pose(self, data):
        self.ta_current_x = data.data[0]
        self.ta_current_y = data.data[1]
        self.ta_current_z = data.data[2]

    def get_ar_pose(self, msg):
        self.ar_pose_array.fill(0)
        for i in range(len(msg.markers)):
            self.ar_pose_array[msg.markers[i].id][0] = round(msg.markers[i].pose.pose.position.x*1000, 2)
            self.ar_pose_array[msg.markers[i].id][1] = round(msg.markers[i].pose.pose.position.y*1000, 2)
            self.ar_pose_array[msg.markers[i].id][2] = round(msg.markers[i].pose.pose.position.z*1000, 2)

            self.ar_pose_array[msg.markers[i].id][3] = msg.markers[i].pose.pose.orientation.w
            self.ar_pose_array[msg.markers[i].id][4] = msg.markers[i].pose.pose.orientation.x
            self.ar_pose_array[msg.markers[i].id][5] = msg.markers[i].pose.pose.orientation.y
            self.ar_pose_array[msg.markers[i].id][6] = msg.markers[i].pose.pose.orientation.z

        if self.TA_Vision_Mqtt.TD_Vision_ID == 1 and self.ar_pose_array[7][2] != 0.0:
            self.TA_Vision_Mqtt.TD_Vision_ID = 0

            target_pose =[self.ta_current_x - self.ar_pose_array[7][0] + self.x1_dis, self.ta_current_y + self.y1_dis + self.ar_pose_array[7][1], self.ta_current_z - self.ar_pose_array[7][2] + self.z1_dis]
            target_pose1 =[self.ta_current_x - self.ar_pose_array[7][0] + self.x2_dis, self.ta_current_y + self.y2_dis + self.ar_pose_array[7][1], self.ta_current_z - self.ar_pose_array[7][2] + self.z2_dis]

            self.ta_vision_req_msg["action"] = self.TA_Vision_Mqtt.TD_Vision_Type
            self.ta_vision_req_msg["target_pose"] = target_pose #[self.ar_pose_array[0][0], self.ar_pose_array[0][1], self.ar_pose_array[0][2]]
            self.ta_vision_req_msg["target_pose1"] = target_pose1 #[self.ar_pose_array[0][0], self.ar_pose_array[0][1], self.ar_pose_array[0][2]]

            self.ta_vision_req_json = json.dumps(self.ta_vision_req_msg)
            self.TA_Vision_Mqtt.on_publish(self.ta_vision_req_json, "/HG_DEV/Vision_REQ", 2)
            # data = [self.ar_pose_array[0][0], self.ar_pose_array[0][1], self.ar_pose_array[0][2]]
            target_pose_data = Float32MultiArray(data=target_pose)
            target_pose_data1 = Float32MultiArray(data=target_pose1)
            for i in range(4):
                self.ta_target_pose.publish(target_pose_data)
                self.ta_target_pose1.publish(target_pose_data1)

        if self.TA_Vision_Mqtt.TD_Vision_ID == 2 and self.ar_pose_array[1][2] != 0.0:
            self.TA_Vision_Mqtt.TD_Vision_ID = 0
            self.ta_vision_req_msg["action"] = self.TA_Vision_Mqtt.TD_Vision_Type
            target_pose = [self.ta_current_x - self.ar_pose_array[1][0] + self.x1_dis, self.ta_current_y + self.y1_dis + self.ar_pose_array[1][1], -(self.ta_current_z - self.ar_pose_array[1][2] + self.z1_dis)]
            target_pose1 = [self.ta_current_x - self.ar_pose_array[1][0] + self.x2_dis, self.ta_current_y + self.y2_dis + self.ar_pose_array[1][1], -(self.ta_current_z - self.ar_pose_array[1][2] + self.z2_dis)]

            self.ta_vision_req_msg["target_pose"] = target_pose #[self.ar_pose_array[1][0], self.ar_pose_array[1][1], self.ar_pose_array[1][2]]
            self.ta_vision_req_msg["target_pose1"] = target_pose1

            self.ta_vision_req_json = json.dumps(self.ta_vision_req_msg)
            self.TA_Vision_Mqtt.on_publish(self.ta_vision_req_json, "/HG_DEV/Vision_REQ", 2)
            # data = [self.ar_pose_array[0][0], self.ar_pose_array[0][1], self.ar_pose_array[0][2]]
            target_pose_data = Float32MultiArray(data=target_pose)
            target_pose_data1 = Float32MultiArray(data=target_pose1)
            for i in range(4):
                self.ta_target_pose.publish(target_pose_data)
                self.ta_target_pose1.publish(target_pose_data1)

        if self.TA_Vision_Mqtt.TD_Vision_ID == 3 and self.ar_pose_array[2][2] != 0.0:
            self.TA_Vision_Mqtt.TD_Vision_ID = 0
            self.ta_vision_req_msg["action"] = self.TA_Vision_Mqtt.TD_Vision_Type
            target_pose = [self.ta_current_x - self.ar_pose_array[2][0] + self.x1_dis, self.ta_current_y + self.y1_dis + self.ar_pose_array[2][1], -(self.ta_current_z - self.ar_pose_array[2][2] + self.z1_dis)]
            target_pose1 = [self.ta_current_x - self.ar_pose_array[2][0] + self.x2_dis, self.ta_current_y + self.y2_dis + self.ar_pose_array[2][1], -(self.ta_current_z - self.ar_pose_array[2][2] + self.z2_dis)]

            self.ta_vision_req_msg["target_pose"] = target_pose #[self.ar_pose_array[2][0], self.ar_pose_array[2][1], self.ar_pose_array[2][2]]
            self.ta_vision_req_msg["target_pose1"] = target_pose1
            self.ta_vision_req_json = json.dumps(self.ta_vision_req_msg)
            self.TA_Vision_Mqtt.on_publish(self.ta_vision_req_json, "/HG_DEV/Vision_REQ", 2)
            # data = [self.ar_pose_array[0][0], self.ar_pose_array[0][1], self.ar_pose_array[0][2]]
            target_pose_data = Float32MultiArray(data=target_pose)
            target_pose_data1 = Float32MultiArray(data=target_pose1)
            for i in range(4):
                self.ta_target_pose.publish(target_pose_data)
                self.ta_target_pose1.publish(target_pose_data1)
        
        if self.vision_mode == 1 and self.ar_pose_array[6][2] != 0.0 and self.push_state == 1:
            target_pose = [self.ta_current_x - self.ar_pose_array[6][0] + self.x1_dis, self.ta_current_y + self.y1_dis + self.ar_pose_array[6][1], self.ta_current_z - self.ar_pose_array[6][2] + z1_dis]
            target_pose1 = [self.ta_current_x - self.ar_pose_array[6][0] + self.x2_dis, self.ta_current_y + self.y2_dis + self.ar_pose_array[6][1], self.ta_current_z - self.ar_pose_array[6][2] + z2_dis]

            self.ta_vision_req_msg["action"] = self.TA_Vision_Mqtt.TD_Vision_Type
            self.ta_vision_req_msg["target_pose"] = target_pose#[self.ar_pose_array[0][0], self.ar_pose_array[0][1], self.ar_pose_array[0][2]]
            self.ta_vision_req_msg["target_pose1"] = target_pose1
            self.ta_vision_req_json = json.dumps(self.ta_vision_req_msg)
            self.TA_Vision_Mqtt.on_publish(self.ta_vision_req_json, "/HG_DEV/Vision_REQ", 2)
            # data = [self.ar_pose_array[0][0], self.ar_pose_array[0][1], self.ar_pose_array[0][2]]
            target_pose_data = Float32MultiArray(data=target_pose)
            target_pose_data1 = Float32MultiArray(data=target_pose1)
            for i in range(4):
                self.push_target_pose.publish(target_pose_data)
                self.push_target_pose1.publish(target_pose_data1)
            
            self.push_state = 0

if __name__ == '__main__':
    ta_start = TA_Vision()
    rospy.spin()
