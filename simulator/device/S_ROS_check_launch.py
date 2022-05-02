#!/usr/bin/env python3

import time
import xml.etree.ElementTree as ET

import rospy
import rosnode
from necst.msg import String_list_msg


def launch_check():
    launch = ET.parse("/home/amigos/ros/src/necst/simulator/launch/simulator.launch")
    root = launch.getroot()
    return [child.attrib["name"] for child in root]


def node_check():
    print("start function")
    launch = launch_check()
    flag = False
    while not rospy.is_shutdown():
        names = rosnode.get_node_names()
        no_alive = [i for i in launch if not "/" + str(i) in names]
        if (no_alive == []) and (flag is True):
            print("all node is move")
            pub.publish(no_alive, node_name, time.time())
            flag = False
            pass
        elif no_alive:
            # rospy.logfatal(no_alive)
            pub.publish(no_alive, node_name, time.time())
            flag = True
        else:
            pass
        time.sleep(1.0)
    return


node_name = "check_launch"
rospy.init_node(node_name)
pub = rospy.Publisher("check_launch", String_list_msg, queue_size=1)
node_check()
