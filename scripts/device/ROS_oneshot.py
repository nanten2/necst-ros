#!/usr/bin/env python3

import rospy
import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist_packages")
import time
import threading
from necst.msg import oneshot_msg

class oneshot(object):
    
    def __init__(self):
        #rospy.init_node('oneshot_con')
        pass
    
    def oneshot(self, filename, dirname, shot_mode):
        msg = oneshot_msg()
        msg.filename = filename
        msg.dirname = dirname
        msg.shot_mode = shot_mode
        pub = rospy.Publisher('oneshot', oneshot_msg, queue_size=1, latch=True)
        time.sleep(0.1)
        
        for i in range(100):
            pub.publish(msg)
            time.sleep(0.1)
        return

if __name__=='__main__':
    #rospy.init_node('oneshot_con')
    one = oneshot()
    one.oneshot(filename,dirname)
    
    
