#!/usr/bin/env python3

import time
from datetime import datetime as dt
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/necst/ros/src/necst/lib")
import rospy
from necst.msg import Status_timer_msg

node_name = "timer"
import topic_status
deco = topic_status.deco(node_name)

@deco
def time_publish():
    msg = Status_timer_msg()
    while not rospy.is_shutdown():
        tv = time.time()
        mjd = tv/24./3600. + 40587.0 # 40587.0 = MJD0                                                        
        
        ntime = dt.utcnow()
        secofday = ntime.hour*60*60 + ntime.minute*60 + ntime.second + ntime.microsecond*0.000001
        lst_g = 0.67239+1.00273781*(mjd-40000.0)
        l_plb = -67.7222222222/360.0
        lst_plb = lst_g + l_plb
        lst_plb_i = int(lst_plb)
        lst_plb -= lst_plb_i
        lst_plb = 24.0*lst_plb
        lst_hh = int(lst_plb)
        lst_plb = 60.0*(lst_plb - lst_hh)
        lst_mm = int(lst_plb)
        lst_plb = 60.0*(lst_plb -lst_mm)
        lst_ss = int(lst_plb)
        #lst_hh = "{0:02d}".format(lst_hh)
        #lst_mm = "{0:02d}".format(lst_mm)
        #lst_ss = "{0:02d}".format(lst_ss)
        #print(lst_hh,':',lst_mm,':',lst_ss)

        msg.secofday = secofday
        msg.lst_h = lst_hh
        msg.lst_m = lst_mm
        msg.lst_s = lst_ss
        msg.utc_Y = ntime.year
        msg.utc_M = ntime.month
        msg.utc_D = ntime.day
        msg.utc_h = ntime.hour
        msg.utc_m = ntime.minute
        msg.utc_s = ntime.second
        msg.mjd = mjd
        msg.from_node = node_name
        msg.timestamp = time.time()
        try:
            pub.publish(msg)
        except Exception as e:
            print(e)
        time.sleep(0.1)

if __name__=="__main__":
    rospy.init_node(node_name)
    pub = rospy.Publisher("timer", Status_timer_msg, queue_size=1)
    time_publish()
    
