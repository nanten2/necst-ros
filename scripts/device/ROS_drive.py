#! /usr/bin/env python3

"""
------------------------------------------------
[History]
2017/10/05 : kondo takashi
2017/12/14 : kondo
------------------------------------------------
"""
import sys
import time
import rospy
import threading
import sys
sys.path.append("/home/amigos/ros/src/necst/lib/device")
sys.path.append("/home/necst/ros/src/necst/lib/device")
import drive_device as dev

from necst.msg import String_necst

node_name = "drive"

class drive(object):

    switch = {"drive" : "", "contactor" : "" }

    
    def __init__(self):
        """initialize"""
        rospy.Subscriber("antenna_drive", String_necst, self._drive, queue_size=1)
        rospy.Subscriber("antenna_contactor", String_necst, self._contactor, queue_size=1)
        return

    def _drive(self, req):
        self.switch["drive"] = req.data
        return 

    def _contactor(self, req):
        self.switch["contactor"] = req.data
        return 
    
    def move(self):
        while not rospy.is_shutdown():
            if self.switch["drive"]:
                dev.move_drive(self.switch["drive"])
                self.switch["drive"] = ""
            else:
                pass
            if self.switch["contactor"]:
                dev.move_contactor(self.switch["contactor"])
                self.switch["contactor"] = ""
            else:
                pass
            time.sleep(0.1)
        return

    
if __name__ == "__main__":
    rospy.init_node(node_name)
    rospy.loginfo("ROS_drive start.")
    dr = drive()
    dr.move()

