#! /usr/bin/env python

"""
------------------------------------------------
[History]
2017/10/05 : kondo takashi
------------------------------------------------
"""
import time
import rospy
from std_msgs.msg import String
from necst.msg import Status_drive_msg
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
import gpg2000_board
#import test_board # antenna_board test

class drive(object):

    drive_param = 0 #test 
    contactor_param = 0 #test 

    def __init__(self):
        self.bd = gpg2000_board.board()
        #self.board = test_board.board()# test
        self.pub = rospy.Publisher('status_drive', Status_drive_msg, queue_size=10, latch=True)#test
        self.msg = Status_drive_msg()#test
        
    def drive(self, req):
        if req.data == "on":
            _drive = 1#test
            print("drive_start")
            ret = self.bd.out_byte("FBIDIO_OUT1_8", 3)
            print("drive_on")
        elif req.data == "off":
            _drive = 0#test
            ret = self.bd.out_byte("FBIDIO_OUT1_8", 0)
        else:
            print(req.data)
            rospy.logerr('bad command !!')
            ret = False
        if True:#ret:
            rospy.loginfo("complete !!")
        else:
            rospy.logerr("board_drive is unfinished !!")
    
    def contactor(self, req):
        if req.data == "on":
            _contactor = 1#test
            ret = self.bd.out_byte("FBIDIO_OUT9_16", 15)
            print("contactor_on")
        elif req.data == "off":
            _contactor = 0#test
            ret = self.bd.out_byte("FBIDIO_OUT9_16", 0)
        else:
            rospy.logerr('bad command !!')
        if True:#ret:
            rospy.loginfo("complete !!")
        else:
            rospy.logerr("board_drive is unfinished !!")
            
    """limit_check.py
    def drive_pub(self):
        pub = rospy.Publisher('status_drive', Status_drive_msg, queue_size=10, latch=True)
        msg = Status_drive_msg()
        flag=True
        print(self.drive_param)
        while flag:
            try:
                drive1 = self.board.in_byte('FBIDIO_IN1_8')
                drive2 = self.board.in_byte('FBIDIO_IN9_16')
                drive3 = self.board.in_byte('FBIDIO_IN17_24')#dome
                drive4 = self.board.in_byte('FBIDIO_IN25_32')#dome
                msg.name = ['FBIDIO_IN1_8', 'FBIDIO_IN9_16']#,'FBIDIO_IN17_24', 'FBIDIO_IN25_32']
                #msg.value = [drive1, drive2, drive3, drive4]
                msg.value = [int(self.drive_param), int(self.contactor_param)]#, "dome_drive", "memb_drive"]
                print(msg.value)
                pub.publish(msg)
                time.sleep(1)


            except:
                rospy.logerr("no publish !!")
                flag=False
                break
    """

if __name__ == "__main__":
    rospy.init_node("drive")
    rospy.loginfo("ROS_drive start.")
    dr = drive()
    rospy.Subscriber("antenna_drive", String, dr.drive)
    rospy.Subscriber("antenna_contactor", String, dr.contactor)
    #dr.drive_pub()
    rospy.spin()

