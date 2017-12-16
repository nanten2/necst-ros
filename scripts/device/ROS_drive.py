#! /usr/bin/env python3

"""
------------------------------------------------
[History]
2017/10/05 : kondo takashi
2017/12/14 : kondo
------------------------------------------------
"""
import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import time
import rospy
from std_msgs.msg import String
from necst.msg import Status_drive_msg
sys.path.append("/home/necst/ros/src/necst/lib")

import pyinterface

class drive(object):

    drive_param = 0 #test 
    contactor_param = 0 #test 

    def __init__(self):
        board_name = 2724
        rsw_id = 1
        self.dio = pyinterface.open(board_name, rsw_id)
        self.dio.initialize()
        #self.board = test_board.board()# test
        self.pub = rospy.Publisher('status_drive', Status_drive_msg, queue_size=10, latch=True)#test
        self.msg = Status_drive_msg()#test
        
    def drive(self, req):
        ret = ""
        if req.data == "on":
            ret = self.dio.output_point([1,1], 1) #output_byte([1,1,0,0,0,0,0,0],OUT1_8)
            print("drive_on")
        elif req.data == "off":
            ret = self.dio.output_point([0,0], 1) #output_byte([0,0,0,0,0,0,0,0], 'OUT1_8')
            print("drive_off")
        else:
            rospy.logerr('bad command !!')
            print(req.data)
            ret = False
        if ret:
            rospy.loginfo("complete !!")
        else:
            rospy.logerr("drive_change is unfinished !!")
    
    def contactor(self, req):
        ret = ""
        if req.data == "on":
            ret = self.dio.output_point([1,1,1,1], 9) #output_byte([1,1,1,1,0,0,0,0], 'OUT9_16')
            print("contactor_on")
        elif req.data == "off":
            ret = self.dio.output_point([0,0,0,0], 9) #output_byte([0,0,0,0,0,0,0,0], 'OUT9_16')
            print("contactor_off")
        else:
            rospy.logerr('bad command !!')
            print(req.data)
            ret = False
        if ret:
            rospy.loginfo("complete !!")
        else:
            rospy.logerr("contactor_change is unfinished !!")

if __name__ == "__main__":
    rospy.init_node("drive")
    rospy.loginfo("ROS_drive start.")
    dr = drive()
    rospy.Subscriber("antenna_drive", String, dr.drive)
    rospy.Subscriber("antenna_contactor", String, dr.contactor)
    rospy.spin()

