#!/usr/bin/env python3

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
    # print(req)
    # dfs = achilles.dfs()
    # data = dfs.oneshot(req.repeat, req.exposure, req.stime)
    data = [
        [
            [10000.0 + np.random.randint(100) for i in range(16384)]
            for i in range(int(req.repeat))
        ],
        [
            [20000.0 + np.random.randint(100) for i in range(16384)]
            for i in range(int(req.repeat))
        ],
    ]
    data0 = []
    data1 = []
    calc0 = [data0.extend(i) for i in list(data[0])]
    calc1 = [data1.extend(i) for i in list(data[1])]

    time.sleep(req.exposure * req.repeat)
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
