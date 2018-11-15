#!/usr/bin/env python3

import sys
import time
import rospy
import ROS_controller
con = ROS_controller.controller()

cc = input("Please input sleep time[s] :")

try:
    cc = int(cc)
except:
    print("no integer...")
    sys.exit()

while not rospy.is_shutdown():
    con.drive("on")
    time.sleep(0.1)
    con.drive("off")
    time.sleep(cc)
