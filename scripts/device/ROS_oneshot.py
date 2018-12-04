#!/usr/bin/env python3

import rospy
import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist_packages")
import time
import threading
from necst.msg import oneshot_msg

class oneshot(object):
    time = 0
    
    def __init__(self):
        self.time = time.time()
        pass
    
    def oneshot(self):
        msg = oneshot_msg()
        msg.time = self.time
        pub = rospy.Publisher('oneshot', oneshot_msg, queue_size=1, latch=True)
        time.sleep(0.1)
        pub.publish(msg)
        
        '''
        while not rospy.is_shutdown():
            pub.publish(msg)
            print('oneshot!!')
            time.sleep(0.1)
        '''
        return

if __name__=='__main__':
    rospy.init_node('oneshot_con')
    one = oneshot()
    one.oneshot()
    
    
