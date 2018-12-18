#!/usr/bin/env python3

from datetime import datetime as dt
import time
import os
import rospy
from necst.msg import String_list_msg

def _get_data(req):
    if float(req.data[2]) == 0:
        return
    stime = (float(req.data[2])-40587)*24*3600
    etime = stime + float(req.data[0])*float(req.data[1])
    f.write(str(time.time())+" "+str(stime)+" "+str(etime)+"\n")
    print("save_file : ", save_dir+file_name)
    return
    
if __name__ == "__main__":
    rospy.init_node("record_ac240_time")    
    now = dt.utcnow()
    save_dir = "/home/amigos/data/experiment/ac240_time/"
    if not os.path.isdir(save_dir):
        os.makedirs(save_dir)
    file_name = now.strftime("ac240_%Y%m%d_%H%M%S.txt")
    f = open(save_dir+file_name, "a")
    sub = rospy.Subscriber("ac240_get_data",String_list_msg, _get_data)
    rospy.spin()
