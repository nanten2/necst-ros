#!/usr/bin/env python
import rospy
import time
from necst.msg import xffts_flag_msg

rospy.init_node("obsmode")

data = ""

def callback(req):
    global data
    data = req

sub = rospy.Subscriber("XFFTS_DB_flag", xffts_flag_msg, callback, queue_size=10)
pub = rospy.Publisher("obsmode", xffts_flag_msg, queue_size=10)

d = xffts_flag_msg()

while not rospy.is_shutdown():
    if data == "":
        time.sleep(0.1)
        continue
    d.scan_num = data.scan_num
    d.obs_mode = "{:10s}".format(data.obs_mode)
    d.lamdel = data.lamdel
    d.betdel = data.betdel
    pub.publish(d)
    time.sleep(0.05)
