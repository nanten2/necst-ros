#!/usr/bin/env python3

import time
import json
import rospy
import rosnode
from necst.msg import Bool_necst
from necst.msg import String_necst

node_name = "check_alert"
stop_node = []

def _regist(req):
    global stop_node
    if not "/" in req.from_node:
        req.from_node = "/"+req.from_node
    if req.data == True and (not req.from_node in stop_node):
        stop_node.append(req.from_node)
    elif req.data == False and (req.from_node in stop_node):
        stop_node.remove(req.from_node)
    else:
        pass
    print("stop list : ", stop_node)
    return

def check_alert_node():
    error_flag = False
    time.sleep(2.)
    node_data = rosnode.get_node_names()
    start_node = [i for i in node_data if i.startswith("/alert")]
    print("start_alert : ", start_node)
    while not rospy.is_shutdown():
        node_data = rosnode.get_node_names()
        move_node = [i for i in node_data if i.startswith("/alert")]
        check_node = list(set(start_node)-set(start_node and stop_node))
        diff = set(check_node)-set(move_node)
        if not diff and error_flag == True:
            rospy.loginfo("All alert start moving !!")
            error_flag = False
        elif not diff:
            pass
        else:
            print(check_node, move_node)
            
            con.move_stop()
            con.dome_stop()
            con.memb_close()
            con.dome_close()
            rospy.logfatal("Alert node is some down...")
            rospy.logfatal(str(diff))
            error_flag = True
        current_node = {"true": list(set(move_node)-set(stop_node)),
                        "false":list(stop_node)}
        bb = json.dumps(current_node)
        pub.publish(data=bb, from_node=node_name, timestamp=time.time())
        time.sleep(0.1)

if __name__ == "__main__":
    import ROS_controller
    con = ROS_controller.controller(escape=node_name)
    rospy.loginfo("check start!! : ROS_alert.py ")
    sub = rospy.Subscriber("stop_alert", Bool_necst, _regist, queue_size=1)
    pub = rospy.Publisher("check_alert", String_necst, queue_size=1)
    time.sleep(5.)
    check_alert_node()
    
