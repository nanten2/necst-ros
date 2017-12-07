#!/usr/bin/env python3

import rospy
import time
rospy.init_node('test_python2_or_3')
while not rospy.is_shutdown():
    print(7/3)
    time.sleep(1)
rospy.spin()
