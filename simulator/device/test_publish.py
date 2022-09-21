#!/usr/bin/env python3

import rospy
import time
from necst.msg import Float64_necst

def talker():
    msg = Float64_necst()
    pub = rospy.Publisher('test', Float64_necst, queue_size=1, latch = True)
    rospy.init_node('Publisher')
     
    while not rospy.is_shutdown():
        msg.timestamp = time.time()
        msg.from_node = 'publisher'
        msg.data = time.time()
        pub.publish(msg)
        print('publish')
        time.sleep(0.01)
 
if __name__ == '__main__':
    talker()
