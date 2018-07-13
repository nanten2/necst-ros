#!/usr/bin/env python3

import time
import ast
import rospy
import rosnode
from necst.msg import String_list_msg

def launch_check():
    f = open("/home/amigos/ros/src/necst/simulator/launch/simulator.launch","r")
    _line = f.readlines()
    f.close()
    del _line[0]
    del _line[-1]
    launch = [ast.literal_eval(i.split()[2].split("=")[1]) for i in _line]
    return launch
 
def node_check():
    print("start function")
    launch = launch_check()
    flag = False
    while not rospy.is_shutdown():
        names = rosnode.get_node_names()
        no_alive = [ i for i in launch if not "/"+str(i) in names]
        if no_alive == [] and flag==True:
            print("all node is move")
            pub.publish(no_alive, node_name, time.time())            
            flag = False
            pass
        elif no_alive:
            rospy.logfatal(no_alive)
            pub.publish(no_alive, node_name, time.time())
            flag = True
        else:
            pass
        time.sleep(1.)
    return 

node_name = "check_launch"
rospy.init_node(node_name)
pub = rospy.Publisher("check_launch", String_list_msg, queue_size=1)
node_check()
