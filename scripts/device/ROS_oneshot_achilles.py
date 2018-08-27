#! /usr/bin/env python2.7

import rospy
import achilles
import time
from necst.srv import ac240_srv
from necst.srv import ac240_srvResponse

def oneshot(req):
    print("start oneshot")
    print(req)
    dfs = achilles.dfs()
    data = dfs.oneshot(req.repeat, req.exposure, req.stime)

    print("fin")
    return ac240_srvResponse(data[0], data[1])

if __name__ == "__main__":
    rospy.init_node("achilles")
    print("start\n")
    sub = rospy.Service("ac240",ac240_srv, oneshot)
    rospy.spin()
