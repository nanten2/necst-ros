#!/usr/bin/env python3

import numpy
import rospy
import rosnode
import time
import threading
import ast

from necst.msg import Status_node_msg

node = {}
launch = ""
no_alive = ""

def launch_check():
    f = open("/home/amigos/ros/src/necst/simulator/launch/simulator.launch","r")
    _line = f.readlines()
    f.close()
    del _line[0]
    del _line[-1]
    launch = [ast.literal_eval(i.split()[2].split("=")[1]) for i in _line]
    return launch
 
def node_check():
    global no_alive
    launch = launch_check()
    while not rospy.is_shutdown():
        names = rosnode.get_node_names()
        no_alive = [ i for i in launch if not "/"+str(i) in names]
        if no_alive == []:
            no_alive = ""
        elif no_alive:
            rospy.logfatal(no_alive)
        else:
            pass
        time.sleep(0.5)
    return 

def _topic(req):
    node[req.from_node] = req.active
    return

def pub_node():
    msg = Status_node_msg()
    while not rospy.is_shutdown():
        cnode = node
        print("cnode",cnode)
        for i in cnode.keys():
            if i in no_alive:
                time.sleep(1.)
                if i in no_alive:
                    cnode[i] = False
                else:
                    pass
            else:
                pass            
            msg.from_node = i
            msg.active = cnode[i]
            pub.publish(msg)
            print(msg)
            print("\n")
        time.sleep(1.)
    
rospy.init_node("topic_server")
sub1 = rospy.Subscriber("status_topic", Status_node_msg, _topic, queue_size=1)
pub = rospy.Publisher("web_topic", Status_node_msg, queue_size=1)
node_thread = threading.Thread(target=node_check)
node_thread.start()
pub_node()
