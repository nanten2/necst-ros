#!/usr/bin/env python3

import time
import rospy
from necst.msg import Status_node_msg

node_name = "check_node"
topic_dict = {}

def _topic(req):
    global topic_dict
    topic_dict[req.from_node] = req.active
    return

def pub_topic():
    msg = Status_node_msg()
    while not rospy.is_shutdown():
        ctopic = topic_dict
        for i in list(ctopic):
            msg.from_node = i
            msg.active = ctopic[i]
            pub.publish(msg)
            time.sleep(0.1)
        time.sleep(1)
            

rospy.init_node(node_name)
sub = rospy.Subscriber("status_topic", Status_node_msg, _topic, queue_size=1)
pub = rospy.Publisher("check_node", Status_node_msg, queue_size=1)
pub_topic()
