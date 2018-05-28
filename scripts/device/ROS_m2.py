#!/usr/bin/env python3

"""
create : okuda
correction : kondo 2017/12/19
"""

import sys
import rospy
import time
import threading

sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("home/necst/ros/src/necst/lib")
import m2_device as dev

from necst.msg import String_necst
from necst.msg import Float64_necst
from necst.msg import Int64_necst

node_name = 'm2_controller'

class m2_controller(object):
    #reference  /src/obs/modules/hal/subref.cpp
    #micron is controlled by time
    
    status = [0.0, 1, 1]#[m_pos, m_limit_up, m_limit_down]
    puls = ""
    
    def __init__(self):
        dev.get_pos()
        self.sub1 = rospy.Subscriber('m2', Int64_necst, self._convert)
        self.sub2 = rospy.Subscriber('emergency', String_necst, self._emergency)
        self.pub = rospy.Publisher('status_m2',Float64_necst, queue_size=10, latch = True)        
        return

    # --------
    # move
    # --------

    def _convert(self, req):
        #move subref
        self.puls = dev.um_to_puls(req.data, self.status)
        return

    def move(self):
        while not rospy.is_shutdown():
            if not self.puls:
                pass
            else:
                print("move start")
                dev.MoveIndexFF(self.puls)
                self.puls = ""
                print("move end")
            time.sleep(0.1)
        return    
    
    def _emergency(self,req):
        rospy.loginfo('!!!emergency!!!')
        rospy.loginfo('!!!stop M2!!!')
        self.emergency_flag = 1
        return

    # --------
    # status
    # --------
    

    def status_thread(self):
        status = threading.Thread(target = self.pub_status)
        status.setDaemon(True)
        status.start()
        return
    
    def pub_status(self):
        msg = Float64_necst()
        msg.from_node = node_name        
        while not rospy.is_shutdown():
            self.status = dev.get_pos()
            msg.data = self.status[0]
            msg.timestamp = time.time()
            self.pub.publish(msg)
            rospy.loginfo(msg)
            time.sleep(0.1)
        return

if __name__ == '__main__':
    rospy.init_node(node_name)
    m2 = m2_controller()
    rospy.loginfo('waiting publish M2')
    m2.status_thread()
    m2.move()

