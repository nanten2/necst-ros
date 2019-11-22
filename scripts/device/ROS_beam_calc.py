#! /usr/bin/env python3

import rospy
from necst.msg import Status_encoder_msg
from necst.msg import Center_beam_num_msg
from necst.msg import Center_beam_position_msg
from necst.msg import String_necst

from datetime import datetime
from astropy.time import Time
import time
import sys
sys.path.append("/home/amigos/ros/src/necst/lib/")
sys.path.append("/home/necst/ros/src/necst/lib/")

node_name = 'beam_calc'

class beam_calc(object):

    enc_az = 0
    enc_el = 0
    enc_time = 0
    center_beam = 1

    def __init__(self):
        rospy.Subscriber('status_encoder', Status_encoder_msg, self.callback1, queue_size=1)
        rospy.Subscriber('center_beam_num', Center_beam_num_msg, self.callback2, queue_size=1)
        self.pub = rospy.Publisher('center_beam_position', Center_beam_position_msg, queue_size=1)
        ###Model
        self.beam3az = -296.25
        self.beam3el = 301.31
        self.beam3aztheta = 92.81
        self.beam3eltheta = -2.25

        self.beam4az = -300.30
        self.beam4el = -356.60
        self.beam4aztheta = -5.40
        self.beam4eltheta = 83.08 

        self.beam5az = 387.92
        self.beam5el = -341.73
        self.beam5aztheta = 98.15
        self.beam5eltheta = 2.32
        pass

    def callback1(self, req):
        self.enc_az = req.enc_az
        self.enc_el = req.enc_el
        self.enc_time = req.timestamp
        return

    def callback2(self, req):
        self.center_beam = req.center_beam
        return
    
    def calc(self):
        while not rospy.is_shutdown():
            msg = Center_beam_position_msg()
            if self.center_beam == 1:
                ddx = 0
                ddy = 0
                
            elif self.center_beam == 2:
                ddx = 60*5.35*(-np.sin(np.radians(-self.enc_el)))
                ddy = 60*5.35*(np.cos(np.radians(-self.enc_el)))
                
            elif self.center_beam == 3:
                ddx = self.beam3az*(np.sin(np.radians(self.enc_el + self.beam3aztheta)))
                ddy = self.beam3el*(np.sin(np.radians(self.enc_el + self.beam3eltheta)))
                #ddx = - 60*5.35*(np.cos(np.radians(-self.enc_el)))
                #ddy = - 60*5.35*(np.sin(np.radians(-self.enc_el)))
                
            elif self.center_beam == 4:
                ddx = - 60*5.35*(-np.sin(np.radians(-self.enc_el)))
                ddy = - 60*5.35*(np.cos(np.radians(-self.enc_el)))
                
            elif self.center_beam == 5:
                ddx = 60*5.35*(np.cos(np.radians(-self.enc_el)))
                ddy = 60*5.35*(np.sin(np.radians(-self.enc_el)))
            else:
                pass
            
            msg.ddx = ddx
            msg.ddy = ddy
            self.pub.publish(msg)
            time.sleep(0.1)
        return
            
if __name__ == '__main__':
    rospy.init_node(node_name)
    calc = beam_calc()
    calc.calc()
