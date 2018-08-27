#! /usr/bin/env python2.7

import os
import rospy
from necst.msg import Achilles_msg
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
import achilles
import numpy as np
import time
from necst.srv import ac240_srv
from necst.srv import ac240_srvResponse

def oneshot(req):
    print("start oneshot")
    print(req)
    #dfs = achilles.dfs()
    #data = dfs.oneshot(req.repeat, req.exposure, req.stime)
    data = [[10000.0+np.random.randint(1000)]*int(req.repeat),
            [20000.0+np.random.randint(1000)]*int(req.repeat)]
    time.sleep(req.exposure*req.repeat)
    print("fin")
    return ac240_srvResponse(data[0], data[1])

if __name__ == "__main__":
    rospy.init_node("achilles")
    print("start\n")
    sub = rospy.Service("ac240",ac240_srv, oneshot)
    rospy.spin()
