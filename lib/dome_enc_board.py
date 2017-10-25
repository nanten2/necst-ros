#!usr/bin/env python

import rospy
import time
#import pyinterface#N
from std_msgs.msg import String


class dome_enc_board(object):
    pub = rospy.Publisher('dome_enc_board', String, queue_size = 10, latch = True)
    dome_enc = 0

    def __init__(self, ndev2 = 1):
        #self.dio = pyinterface.create_gpg6204(ndev2)#N
        #self.ctrl = gpg6204_controller(ndev, initialize=initialize)#N
        pass

    def reset(self):
        #self.dio.ctrl.reset()#N
        return
    
    def set_mode(self):
        #self.dio.ctrl.set_mode()#N
        return
    
    def get_position(self):
        #self.dio.get_position()#N
        print('get position : ROS_dome.py')
        time.sleep(0.1)
        rospy.logfatal(self.dome_enc)
        return self.dome_enc
    
    def set_counter(self, counter):
        #self.dio.ctrl.set_counter(counter)#N
        return
    
    def callback(self, req):
        rospy.loginfo('set')
        self.dome_enc = req.data
        print('dome_enc_board.py',self.dome_enc)
        #time.sleep(0.1)
        return
        
"""
if __name__ == '__main__':
    d = dome_enc_board()
    sub = rospy.Subscriber('dome_enc', String, d.callback)
"""

d = dome_enc_board()
sub = rospy.Subscriber('dome_enc', String, d.callback)

