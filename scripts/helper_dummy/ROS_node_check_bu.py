#!/usr/bin/env python
"""
you can see what node is running now and publishing
2017/10/28 Kazuki Shiotani
"""
import rospy
import rosnode
import time
import threading
from necst.msg import Status_antenna_msg
from necst.msg import Status_weather_msg
from necst.msg import Status_encoder_msg
from necst.msg import Status_dome_msg
from necst.msg import Status_hot_msg
from necst.msg import Status_drive_msg
from necst.msg import Status_m4_msg
from necst.msg import Status_limit_msg
from necst.msg import list_azelmsg
from std_msgs.msg import Bool
from std_msgs.msg import String

OKGREEN = '\033[92m'
FAIL = '\033[91m'
ENDC = '\033[0m'

node_list = ['/Status', '/abs_controller', '/antenna_move', '/antenna_server', '/authority_change', '/dome', '/dome_tracking', '/drive', '/encoder_status', '/limit_check', '/rosout', '/tracking', '/weather_status']


"""
while True:
    print(OKBLUE +  str(rosnode.get_node_names()) + ENDC)
    time.sleep(1)
    continue
"""
class node_check(object):
    #The number of publish
    num1 = 0
    num2 = 0
    num3 = 0
    num4 = 0
    num5 = 0
    num6 = 0
    num7 = 0
    num8 = 0
    num9 = 0
    num10 = 0
    num11 = 0
    num12 = 0
    
    def callback1(self,req):
        self.num1 += 1
    def callback2(self,req):
        self.num2 += 1
    def callback3(self,req):
        self.num3 += 1
    def callback4(self,req):
        self.num4 += 1
    def callback5(self,req):
        self.num5 += 1
    def callback6(self,req):
        self.num6 += 1
    def callback7(self,req):
        self.num7 += 1
    def callback8(self,req):
        self.num8 += 1
    def callback9(self,req):
        self.num9 += 1
    def callback10(self,req):
        self.num10 += 1
    def callback11(self,req):
        self.num11 += 1
    def callback12(self,req):
        self.num12 += 1
        
    def func(self):
        node_data = rosnode.get_node_names()
        node_data = sorted(node_data)
        #print(node_data)
        a = []
        for i in range(len(node_list)):
            if node_list[i] == '/Status':
                script_name = 'ROS_status.py'
                pub_num = '0'
            elif node_list[i] == '/abs_controller':
                script_name = 'ROS_abs.py'
                pub_num = str(self.num5)
            elif node_list[i] == '/authority_change':
                script_name = 'ROS_authority_check.py'
                pub_num = str(self.num12)
            elif node_list[i] == '/weather_status':
                script_name = 'ROS_weather.py'
                pub_num = str(self.num2)
            elif node_list[i] == '/antenna_move':
                script_name = 'ROS_antenna_move.py'
                pub_num = str(self.num1)
            elif node_list[i] == '/encoder_status':
                script_name = 'ROS_encoder.py'
                pub_num = str(self.num3)
            elif node_list[i] == '/dome_tracking':
                script_name = 'ROS_dome_track_check.py'
                pub_num = str(self.num11)
            elif node_list[i] == '/drive':
                script_name = 'ROS_drive.py'
                pub_num = str(self.num6)
            elif node_list[i] == '/tracking':
                script_name = 'ROS_tracking_check.py'
                pub_num = str(self.num10)
            elif node_list[i] == '/dome':
                script_name = 'ROS_dome.py'
                pub_num = str(self.num4)
            elif node_list[i] == '/limit_check':
                script_name = 'ROS_limit_check.py'
                pub_num = str(self.num8)
            elif node_list[i] == '/rosout':
                script_name = 'defalut'
                pub_num = '~~'
            elif node_list[i] == '/antenna_server':
                script_name = 'ROS_antenna.py'
                pub_num = str(self.num9)
            else:
                script_name = 'Unknown'
                pub_num = 'Unknown'
            a.append((script_name, pub_num, node_list[i] in node_data))
            b = ''
            c = ''
            for i in range(len(a)):
                if a[i][2] == True:
                    b += '%s : %s'%(a[i][0], a[i][1])+'\n'
                else:
                    c += '%s : %s'%(a[i][0], a[i][1])+'\n'
        print(time.ctime())
        print('Running Node')
        print('=========================')
        print(OKGREEN + b + ENDC)
        print('Not Running node')
        print('=========================')
        print(FAIL + c + ENDC)

    def start_thread(self):
        th = threading.Thread(target = self.func2)
        th.setDaemon(True)
        th.start()

    def func2(self):        
        while True:
            self.func()
            time.sleep(1)
            continue

if __name__ == '__main__':
    rospy.init_node('publish_check')
    nc = node_check()
    sub1 = rospy.Subscriber('status_antenna', Status_antenna_msg, nc.callback1)
    sub9 = rospy.Subscriber('list_azel', list_azelmsg, nc.callback9)
    sub2 = rospy.Subscriber('status_weather', Status_weather_msg, nc.callback2)
    sub3 = rospy.Subscriber('status_encoder', Status_encoder_msg, nc.callback3)
    sub4 = rospy.Subscriber('status_dome', Status_dome_msg, nc.callback4)
    sub5 = rospy.Subscriber('status_hot', Status_hot_msg, nc.callback5)
    sub6 = rospy.Subscriber('status_drive', Status_drive_msg, nc.callback6)
    sub7 = rospy.Subscriber('status_m4', Status_m4_msg, nc.callback7)
    sub8 = rospy.Subscriber('limit_check', Status_limit_msg, nc.callback8)
    sub10 = rospy.Subscriber("tracking_check", Bool, nc.callback10)
    sub11 = rospy.Subscriber("dome_tracking_check", Bool, nc.callback11)
    sub12 = rospy.Subscriber("authority_check", String, nc.callback12)
    nc.start_thread()
    rospy.spin()
