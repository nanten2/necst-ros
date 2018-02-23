#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import String
from necst.msg import Status_node_msg

def deco(node_name="NONE", message="NONE", frame="print"):
    """node_name=os.path.basename(__file__)"""
    def _deco(func):
        pub = rospy.Publisher("status_node", Status_node_msg, queue_size=1)
        import functools
        @functools.wraps(func)
        def wrapper(*args,**kwargs):
            pub.publish(node_name, frame, message+" is start")
            time.sleep(0.001)
            func(*args,**kwargs)
            pub.publish(node_name, frame, message+" is end")
        return wrapper
    return _deco

