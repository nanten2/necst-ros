#!/usr/bin/env python3

import rospy
import time
import os
from necst.msg import Status_encoder_msg
from necst.msg import Status_antenna_msg
import matplotlib.pyplot as plt
import matplotlib.animation as anime
class plot(object):
    enc_az = 0
    enc_el = 45*3600.
    command_az = 0
    command_el = 45*3600.

    def __init__(self):
        pass

    def callback(self, req):
        self.enc_az = req.enc_az
        self.enc_el = req.enc_el
        return
    
    def callback2(self, req):
        self.command_az = req.command_az
        self.command_el = req.command_el
        return
        
    def plot(self):
        time.sleep(2)
        plt.grid()
        plt.xlabel('Az [deg]')
        plt.ylabel('El [deg]')
        plt.title('AzEl plot')
        while not rospy.is_shutdown():
            az = self.enc_az/3600.
            el = self.enc_el/3600.
            c_az = self.command_az/3600.
            c_el = self.command_el/3600.
            plt.plot(az,el,'yellow', marker='*',markersize=14, markeredgewidth=0.7, markeredgecolor='b' , alpha=0.8)
            plt.plot(c_az,c_el,'blue',marker = '.', markersize=10,alpha=0.3)
            plt.xlim(az-0.05, az+0.05)
            plt.ylim(el-0.05, el+0.05)
            plt.pause(0.8)

            
if __name__ == '__main__':
    plot = plot()
    rospy.init_node('plot')
    sub = rospy.Subscriber('status_encoder', Status_encoder_msg, plot.callback)
    sub1 = rospy.Subscriber('status_antenna', Status_antenna_msg, plot.callback2)
    plot.plot()
