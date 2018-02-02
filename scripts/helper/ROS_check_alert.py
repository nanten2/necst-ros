#!/usr/bin/env python3

import time
import rospy
import rosnode
from std_msgs.msg import String
from necst.msg import Dome_msg

def check_alert_node():
    pub_antenna = rospy.Publisher("move_stop", String, queue_size = 1)
    pub_dome = rospy.Publisher('dome_move', Dome_msg, queue_size = 1 )
    error_flag = False
    while not rospy.is_shutdown():
        node_data = rosnode.get_node_names()
        if ("/alert" in node_data) and error_flag == True:
            rospy.loginfo("ROS_alert.py start moving !!")
            error_flag = False
        elif "/alert" in node_data:
            pass
        else:
            pub_antenna.publish("stop")
            pub_dome.publish(name='command', value='dome_stop')
            pub_dome.publish(name='command', value='memb_close')
            pub_dome.publish(name="command", value='dome_close')
            rospy.logfatal("ROS_alert.py is down...\n\n")
            error_flag = True
        time.sleep(0.1)

if __name__ == "__main__":
    rospy.init_node("check_alert")
    rospy.loginfo("check start!! : ROS_alert.py ")
    time.sleep(5.)
    check_alert_node()
    
