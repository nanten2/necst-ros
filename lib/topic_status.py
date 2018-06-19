#!/usr/bin/env python3

import time
import rospy
from necst.msg import Status_node_msg

pub = rospy.Publisher("status_topic", Status_node_msg, queue_size=1)
name = ""
#import os
#node_name = os.path.basename(__file__)
#node_name = node_name.split(".")[0]

def deco(node_name=""):
    global name
    def _deco(func):
        import functools
        @functools.wraps(func)
        def wrapper(*args,**kwargs):
            pub.publish(True, "start", func.__name__, node_name, time.time())
            name = node_name
            time.sleep(0.1)
            try:
                func(*args,**kwargs)
            except Exception as e:
                pub.publish(False, str(e), func.__name__, node_name, time.time())
                time.sleep(0.1)
                pass
            pub.publish(False, "end", func.__name__, node_name, time.time())
        return wrapper
    return _deco

