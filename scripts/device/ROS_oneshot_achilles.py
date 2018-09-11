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
    dfs = achilles.dfs()
    data = dfs.oneshot(req.repeat, req.exposure, req.stime)
    #data = [
    #    [[[10000.0+np.random.randint(1000) for i in range(16384)]]*int(req.repeat),10],
    #    [[[20000.0+np.random.randint(1000) for i in range(16384)]]*int(req.repeat),20]
    #]
    data0 = []
    data1 = []
    calc0 = [data0.extend(i) for i in data[0][0]]
    calc1 = [data1.extend(i) for i in data[1][0]]            
            
    time.sleep(req.exposure*req.repeat)
    print("fin")
    return ac240_srvResponse(data0, data[0][1], data1, data[1][1])

if __name__ == "__main__":
    rospy.init_node("achilles")
    print("start\n")
    sub = rospy.Service("ac240",ac240_srv, oneshot)
    rospy.spin()
