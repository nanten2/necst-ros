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

from necst.msg import String_necst
from necst.msg import Status_limit_msg
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/necst/ros/src/necst/lib")
import topic_status

node_name = "drive"
deco = topic_status.deco(node_name)

class drive(object):

    switch = {"drive" : "", "contactor" : "" }
    
    def __init__(self):
        """initialize"""
        rospy.Subscriber("antenna_drive", String_necst, self._drive, queue_size=1)
        rospy.Subscriber("antenna_contactor", String_necst, self._contactor, queue_size=1)
        self.limit = rospy.Publisher("limit_check", Status_limit_msg, queue_size=1)
        return

    def _drive(self, req):
        self.switch["drive"] = req.data
        return

    def _contactor(self, req):
        self.switch["contactor"] = req.data
        return

    @deco
    def move(self):
        while not rospy.is_shutdown():
            param = []
            if self.switch["drive"]=="on":
                #dev.move_drive(self.switch["drive"])
                drive = [1,1]
                #self.switch["drive"] = ""
            else:
                drive = [0,0]
                pass
            if self.switch["contactor"]=="on":
                #dev.move_contactor(self.switch["contactor"])
                #self.switch["contactor"] = ""
                contactor = [1,1]
            else:
                contactor = [0,0]
                pass
            param.extend(drive)
            param.extend(contactor)
            param.extend([1]*28)
            self.limit.publish(param, "",  node_name, time.time())
            time.sleep(0.1)
        return
            
if __name__ == "__main__":
    rospy.init_node(node_name)
    rospy.loginfo("ROS_drive start.")
    dr = drive()
    dr.move()

