#!/usr/bin/env python3

import time
import rospy
import rosnode
from necst.msg import Bool_necst
from necst.msg import Dome_msg

node_name = "check_alert"

def check_alert_node():
    pub_antenna = rospy.Publisher("move_stop", Bool_necst, queue_size = 1)
    pub_dome = rospy.Publisher('dome_move', Dome_msg, queue_size = 1 )
    error_flag = False
    time.sleep(2.)
    node_data = rosnode.get_node_names()
    start_node = [i for i in node_data if i.startswith("/alert")]
    print("start_alert : ", start_node)
    while not rospy.is_shutdown():
        node_data = rosnode.get_node_names()
        move_node = [i for i in node_data if i.startswith("/alert")]
        if start_node == move_node and error_flag == True:
            rospy.loginfo("All alert start moving !!")
            error_flag = False
        elif start_node == move_node:
            pass
        else:
            pub_antenna.publish(True, node_name, time.time())
            pub_dome.publish(name='command', value='dome_stop')
            pub_dome.publish(name='command', value='memb_close')
            pub_dome.publish(name="command", value='dome_close')
            rospy.logfatal("Alert node is some down...")
            print("start_node, move_node : ",start_node, move_node)
            error_flag = True
        time.sleep(0.1)

if __name__ == "__main__":
    rospy.init_node(node_name)
    rospy.loginfo("check start!! : ROS_alert.py ")
    time.sleep(5.)
    check_alert_node()
    
