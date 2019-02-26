#!/usr/bin/env python3

import sys
import time
import rospy
import ROS_controller
con = ROS_controller.controller()

tt = input("Please input sleep time[s] :")

try:
    tt = int(tt)
except:
    print("no integer...")
    sys.exit()

while not rospy.is_shutdown():
    con.drive("on")
    time.sleep(tt)
    con.drive("off")
    time.sleep(tt)
