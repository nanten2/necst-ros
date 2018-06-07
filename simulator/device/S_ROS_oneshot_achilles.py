#! /usr/bin/env python2.7

import os
import rospy
from necst.msg import Achilles_msg
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
import achilles
import numpy as np
import time

def oneshot(req):
    print("start oneshot")
    print(req)
    #dfs = achilles.dfs()
    #data = dfs.oneshot(req.repeat, req.exposure, req.stime)
    data = [[10000.0+np.random.randint(1000)]*int(req.repeat),
            [20000.0+np.random.randint(1000)]*int(req.repeat)]
    time.sleep(req.exposure*req.repeat)
    dir_name = "/home/amigos/data/experiment/achilles/" + req.day + "/"
    if not os.path.exists(dir_name):
        os.makedirs(dir_name)
    f1 = open(dir_name+req.day + "_1.txt", "w")
    print(data[0])
    f1.write(str(data[0]))
    f1.close()
    f2 =  open(dir_name+req.day + "_2.txt", "w")
    print(data[1])
    f2.write(str(data[1]))
    f2.close()
    f3 =  open(dir_name+req.day + "_fin.txt", "w")
    f3.write("fin")
    f3.close()
    print("fin")
    return


if __name__ == "__main__":
    rospy.init_node("achilles")
    print("start\n")
    sub = rospy.Subscriber("achilles", Achilles_msg, oneshot)
    rospy.spin()
