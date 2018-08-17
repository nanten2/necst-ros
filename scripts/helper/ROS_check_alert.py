#!/usr/bin/env python3

import time
import rospy
import rosnode

node_name = "check_alert"

def check_alert_node():
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
            con.move_stop()
            con.dome_stop()
            con.memb_close()
            con.dome_close()
            rospy.logfatal("Alert node is some down...")
            print("start_node, move_node : ",start_node, move_node)
            error_flag = True
        time.sleep(0.1)

if __name__ == "__main__":
    import ROS_controller
    con = ROS_controller.controller(escape=node_name)
    rospy.loginfo("check start!! : ROS_alert.py ")
    time.sleep(5.)
    check_alert_node()
    
