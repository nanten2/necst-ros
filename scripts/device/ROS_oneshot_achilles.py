#! /usr/bin/env python3

# Shebang changed from ``python2.7`` (2022/3/16)
# Any reason older Python version was used?

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
from necst.msg import String_list_msg


def oneshot(req):
    print("start oneshot")
    print(req)
    dfs = achilles.dfs()
    data = dfs.oneshot(req.repeat, req.exposure, req.stime)

    data0 = []
    data1 = []
    # print(list(data[0][0]))
    print(len(list(data[0][0])))
    calc0 = [data0.extend(i) for i in list(data[0])]
    calc1 = [data1.extend(i) for i in list(data[1])]

    print("fin")
    pub.publish(
        [str(req.repeat), str(req.exposure), str(req.stime)], "achilles", time.time()
    )
    return ac240_srvResponse(data0, data1)


if __name__ == "__main__":
    rospy.init_node("achilles")
    print("start\n")
    sub = rospy.Service("ac240", ac240_srv, oneshot)
    pub = rospy.Publisher("ac240_get_data", String_list_msg, queue_size=1)
    rospy.spin()
