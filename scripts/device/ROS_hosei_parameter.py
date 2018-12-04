#!/usr/bin/env python3

import os
import time
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/necst/ros/src/necst/lib")
import rospy
from necst.msg import List_coord_msg
from necst.msg import String_list_msg

hosei_file =""
node_name = "hosei_parameter"

def _hosei(req):
    global hosei_file
    hosei_file = req.hosei
    return

rospy.init_node(node_name)
rospy.Subscriber("wc_list", List_coord_msg, _hosei, queue_size=1)
pub = rospy.Publisher("hosei_parameter", String_list_msg, queue_size=1)
latest_time = ""
while not rospy.is_shutdown():
    if not hosei_file:
        time.sleep(1.)
        continue
    else:
        pass
    
    current_time = os.path.getatime("/home/amigos/ros/src/necst/lib/"+hosei_file)
    if latest_time != current_time:
        f = open("/home/amigos/ros/src/necst/lib/"+hosei_file, "r")
        ff = f.readlines()
        parameter = [hosei_file]
        parameter.extend([i.split("\n")[0] for i in ff])
        latest_time = current_time
    else:
        pass
    
    msg = String_list_msg()
    msg.data = parameter#[hosei_file].extend(parameter)
    msg.from_node = node_name
    msg.timestamp = time.time()
    pub.publish(msg)
    time.sleep(1.)

    


    
