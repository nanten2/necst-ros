
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
import threading

import pyinterface

class drive(object):

    contactor_pos = ""
    drive_pos = ""
    contactor_move = ""
    drive_move = ""

    flag = True
    
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
        self.move_thread.setDaemon(True)
        self.move_thread.start()

    def current_position(self):
        pos = self.dio_input.input_byte("IN1_8").to_int()[0:4]
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
        print("Current position is : ")
        print("drive : ",self.drive_pos, "contactor : ", self.contactor_pos)
        return [self.contactor_pos, self.drive_pos]

    def contactor(self, req):
        self.contactor_move = req.data
        return self.contactor_move

    def drive(self, req):
        self.drive_move = req.data
        return self.drive_move
    
    def move(self):
        ret = ""
        while not rospy.is_shutdown():
            #print("drive : ",self.drive_pos, "contactor : ", self.contactor_pos)
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
                pass
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
                pass
            else:
                rospy.logerr('bad command !!')
                print(self.contactor_move)
                ret = False
                
            if ret:
                rospy.loginfo("complete !!")
                ret = ""
            elif ret == "":
                pass
            else:
                rospy.logerr("unfinished !!")
                self.stop_thread.set()
        time.sleep(0.1)

if __name__ == "__main__":
    rospy.init_node("drive")
    rospy.loginfo("ROS_drive start.")
    dr = drive()
    rospy.Subscriber("antenna_drive", String, dr.drive)
    rospy.Subscriber("antenna_contactor", String, dr.contactor)
    rospy.spin()

