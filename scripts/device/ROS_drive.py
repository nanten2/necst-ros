#! /usr/bin/env python3

"""
------------------------------------------------
[History]
2017/10/05 : kondo takashi
2017/12/14 : kondo
------------------------------------------------
"""
import sys
import time
import rospy
from std_msgs.msg import String
from necst.msg import Status_drive_msg
sys.path.append("/home/necst/ros/src/necst/lib")
import threading

import pyinterface

class drive(object):

    contactor_pos = ""
    drive_pos = ""
    contactor_move = ""
    drive_move = ""
    
    def __init__(self):
        """initialize"""
        board_name = 2724
        output_rsw_id = 1
        input_rsw_id = 0
        self.dio = pyinterface.open(board_name, output_rsw_id)
        self.dio_input = pyinterface.open(board_name, input_rsw_id)
        self.pub = rospy.Publisher('status_drive', Status_drive_msg, queue_size=10, latch=True)#test
        self.msg = Status_drive_msg()#test
        self.current_position()
        self.stop_thread = threading.Event()
        self.move_thread = threading.Thread(target = self.move)
        self.move_thread.start()

    def current_position(self):
        pos = self.dio_input.input_byte("IN1_8")[0:4]
        if pos[0] == 1 and pos[1] == 1:
            self.contactor_pos = "on"
        else:
            self.contactor_pos = "off"
            pass
        if pos[2] == 1 and pos[3] == 1:
            self.drive_pos = "on"
        else:
            self.drive_pos = "off"
            pass
        return [self.contactor_pos, self.drive_pos]

    def contactor(self, req):
        self.contacor_move = req.data
        return self.contactor_move

    def drive(self, req):
        self.drive = req.data
        return self.drive_move
    
    def move(self):
        while not rospy.is_shutdown:
            """drive"""
            if self.drive_move == self.drive_pos:
                pass
            elif self.drive_move != self.drive_pos and self.drive_move != "":
                if self.drive_move == "on":
                    ret = self.dio.output_point([1,1], 1) #output_byte([1,1,0,0,0,0,0,0],OUT1_8)
                    self.drive_pos = self.drive_move
                    print("drive_on")
                elif self.drive_move == "off":
                    ret = self.dio.output_point([0,0], 1) #output_byte([0,0,0,0,0,0,0,0], 'OUT1_8')
                    self.drive_pos = self.drive_move
                    print("drive_off")
                else:
                    rospy.logerr('bad command !!')
                    print(self.drive_move)
                    ret = False
            elif self.drive_move == "":
                print("no signal")
            else:
                rospy.logerr('bad command !!')
                print(self.drive_move)
                ret = False

            """contactor"""
            if self.contactor_move == self.contactor_pos:
                pass
            elif self.contactor_move != self.contactor_pos and self.contactor_move != "":
                if self.contactor_move == "on":
                    ret = self.dio.output_point([1,1,1,1], 9) #output_byte([1,1,1,1,0,0,0,0], 'OUT9_16')
                    self.contactor_pos = self.contactor_move
                    print("contactor_on")
                elif self.contactor_move == "off":
                    ret = self.dio.output_point([0,0,0,0], 9) #output_byte([0,0,0,0,0,0,0,0], 'OUT9_16')
                    self.contactor_pos = self.contactor_move
                    print("contactor_off")
                else:
                    rospy.logerr('bad command !!')
                    print(self.contactor_move)
                    ret = False
            elif self.contactor_move == "":
                print("no signal")
            else:
                rospy.logerr('bad command !!')
                print(self.contactor_move)
                ret = False
                
            if ret:
                rospy.loginfo("complete !!")
            else:
                rospy.logerr("unfinished !!")
                self.stop_thread.set()

if __name__ == "__main__":
    rospy.init_node("drive")
    rospy.loginfo("ROS_drive start.")
    dr = drive()
    rospy.Subscriber("antenna_drive", String, dr.drive)
    rospy.Subscriber("antenna_contactor", String, dr.contactor)
    rospy.spin()

