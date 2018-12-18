#!/usr/bin/env python3

import time
import rospy
from necst.msg import Status_node_msg
from necst.msg import String_necst
import json

node_name = "check_node"
topic_dict = {}

def _topic(req):
    global topic_dict
    topic_dict[req.from_node] = req.active
    return

def pub_topic():
    msg = String_necst()
    while not rospy.is_shutdown():
        ctopic = topic_dict
        dump = json.dumps(ctopic)
        msg.from_node = node_name
        msg.timestamp = time.time()
        msg.data = dump
        pub.publish(msg)
        time.sleep(1)
            

rospy.init_node(node_name)
sub = rospy.Subscriber("status_topic", Status_node_msg, _topic, queue_size=1)
pub = rospy.Publisher("check_node", String_necst, queue_size=1)
pub_topic()
