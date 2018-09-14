#!/usr/bin/env python3

import pyinterface
from datetime import datetime as dt
import time
import rospy
from necst.msg import Float64_list_msg

dio = pyinterface.open(2724, 2)
node_name = "dome_dio"


def check_dio():
    while not rospy.is_shutdown():
        now = dt.utcnow()
        date = now.strftime("[UTC] %Y/%m/%d %H:%M:%S")
        parameter = dio.input_point(2, 6)
        print(date, " ", parameter)    
        
        msg = Float64_list_msg()
        msg.data = parameter
        msg.from_node = node_name
        msg.timestamp = time.time()
        pub.publish(msg)
        time.sleep(1.)
    return

if __name__ == "__main__":
    print("check start\n")
    rospy.init_node(node_name)
    pub = rospy.Publisher(node_name, Float64_list_msg, queue_size = 1)
    check_dio()
