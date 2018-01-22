#!/usr/bin/env python3
import time
import sys
sys.path.append("/home/necst/ros/src/necst/lib")
import pyinterface
import rospy
from necst.msg import Status_limit_msg
from std_msgs.msg import Bool
import signal
def handler(num, flame):
    stop_flag = 1
    rospy.is_shutdown()
signal.signal(signal.SIGINT, handler)
            
board_name = 2724
rsw_id = 0
dio = pyinterface.open(board_name, rsw_id)

stop_flag = 0

limit_list = {1:"", 2:"", 3:"", 4:"", 5:"soft limit CW", 6:"soft limit CCW", 7:"soft limit UP", 8:"soft limit DOWN",
              9:"1st limit CW", 10:"1st limit CCW", 11:"1st limit UP", 12:"1st limit DOWN",
              13:"2nd limit CW", 14:"2nd limit CCW", 15:"2nd limit UP", 16: "2nd limit DOWN",
              17:"cable_cw error", 18:"cable_ccw error", 19:"", 20:"",
              21:"az_error", 22:"el_error", 23:"servo_error_az", 24:"servo_error_el",
              25:"emergency_switch"} # ==> bit

rospy.init_node("limit_check")
pub = rospy.Publisher("limit_check", Status_limit_msg, queue_size=10, latch=True)
st = Status_limit_msg()
limit_pub = rospy.Publisher("limit", Bool, queue_size=10, latch=True)

while not rospy.is_shutdown():#stop_flag == 0:
    msg = ""
    ret = dio.input_dword().to_list()
    for i in range(4,16):
        if ret[i] == 0:
            msg = msg+str(limit_list[i+1])
        else:
            pass
    if ret[24] == 0:
        msg = msg+str(limit_list[24+1])
    else:
        pass
    if str(msg):
        print(msg)
        stop_flag = 1

    if stop_flag == 1:
        limit_pub.publish(False)


    st.error_box = ret
    st.error_msg = msg
    pub.publish(st)

    time.sleep(0.1)

