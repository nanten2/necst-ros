#!/usr/bin/env python3

import pyinterface
from datetime import datetime as dt
import time
import threading
import rospy
from necst.msg import Float64_list_msg

dio = pyinterface.open(6204, 0)
node_name = "encoder_dio"

def check_dio(mode, pub, name):
    while not rospy.is_shutdown():
        now = dt.utcnow()
        date = now.strftime("[UTC] %Y/%m/%d %H:%M:%S")
        #parameter = dio.get_mode().to_list()
        if mode == "normal":
            parameter = dio.get_mode().to_list()
        elif mode == "z_mode":
            parameter = dio.get_z_mode().to_list()
        print(date, " ", parameter)                
        msg = Float64_list_msg()
        msg.data = parameter
        msg.from_node = name
        msg.timestamp = time.time()
        pub.publish(msg)
        time.sleep(1.)
    return

if __name__ == "__main__":
    print("check start\n")
    rospy.init_node(node_name)
    name1 = node_name
    name2 = "encoder_z_dio"
    pub1 = rospy.Publisher(name1, Float64_list_msg, queue_size = 1)
    pub2 = rospy.Publisher(name2, Float64_list_msg, queue_size = 1)    

    th1 = threading.Thread(target=check_dio, args=["normal",pub1, name1])
    th1.start()
    th2 = threading.Thread(target=check_dio, args=["z_mode",pub2, name2])
    th2.start()    
