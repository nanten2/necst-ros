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
            
#dio = pyinterface.create_gpg2000(3)
board_name = 2724
rsw_id = 0
dio = pyinterface.open(board_name, rsw_id)

stop_flag = 0
ret = [1]*4
error_box = [1]*32
msg = ""


rospy.init_node("limit_check")
pub = rospy.Publisher("limit_check", Status_limit_msg, queue_size=10, latch=True)
st = Status_limit_msg()
limit_pub = rospy.Publisher("limit", Bool, queue_size=10, latch=True)
_lim = Bool()

while not rospy.is_shutdown():#stop_flag == 0:

    ret[0] = dio.input_byte('IN1_8')
    ret[1] = dio.input_byte('IN9_16')
    ret[2] = dio.input_byte('IN17_24')
    ret[3] = dio.input_byte('IN25_32')
    
    for i in range(8):
        error_box[i] = ret[0][i]
    for i in range(8):
        error_box[i+8] = ret[1][i]
    for i in range(8):
        error_box[i+16] = ret[2][i]
    for i in range(8):
        error_box[i+24] = ret[3][i]
        
    if (error_box[4]) == 0:
        msg = '!!!soft limit CW!!!'
        print(msg)
        stop_flag = 1
    if (error_box[5]) == 0:
        msg = '!!!soft limit CCW!!!'
        stop_flag = 1
        print(msg)
    if (error_box[6]) == 0:
        msg = '!!!soft limit UP!!!'
        stop_flag = 1
        print(msg)
    if (error_box[7]) == 0:
        msg = '!!!soft limit DOWN!!!'
        stop_flag = 1
        print(msg)
    if (error_box[8]) == 0:
        msg = '!!!1st limit CW!!!'
        stop_flag = 1
        print(msg)
    if (error_box[9]) == 0:
        msg = '!!!1st limit CCW!!!'
        stop_flag = 1
        print(msg)
    if (error_box[10]) == 0:
        msg = '!!!1st limit UP!!!'
        stop_flag = 1
        print(msg)
    if (error_box[11]) == 0:
        msg = '!!!1st limit DOWN!!!'
        stop_flag = 1
        print(msg)
    if (error_box[12]) == 0:
        msg = '!!!2nd limit CW!!!'
        stop_flag = 1
        print(msg)
    if (error_box[13]) == 0:
        msg = '!!!2nd limit CCW!!!'
        stop_flag = 1
        print(msg)
    if (error_box[14]) == 0:
        msg = '!!!2nd limit UP!!!'
        stop_flag = 1
        print(msg)
    if (error_box[15]) == 0:
        msg = '!!!2nd limit DOWN!!!'
        stop_flag = 1
        print(msg)

    '''
    if (error_box[16]) == 0:
        msg = "!!!cable_cw : error!!!"
        stop_flag = 1
        print(msg)
    if (error_box[17]) == 0:
        msg = "!!!cable_ccw : error!!!"
        stop_flag = 1
        print(msg)

    """
    if (error_box[18]) == 0:
        msg = '!!!2nd limit DOWN!!!'
        stop_flag = 1
        print(msg)
    if (error_box[19]) == 0:
        msg = '!!!2nd limit DOWN!!!'
        stop_flag = 1
        print(msg)
        """

    if (error_box[20]) == 0:
        msg = '!!!az_error!!!'
        stop_flag = 1
        print(msg)
    if (error_box[21]) == 0:
        msg = '!!!el_error!!!'
        stop_flag = 1
        print(msg)
    if (error_box[22]) == 0:
        msg = '!!!servo_error_az!!!'
        stop_flag = 1
        print(msg)
    if (error_box[23]) == 0:
        msg = '!!!servo_error_el!!!'
        stop_flag = 1
        print(msg)

    '''

    if (error_box[24]) == 0:
        msg = '!!!emergency_switch!!!'
        stop_flag = 1
        print(msg)






    if stop_flag == 1:
        _lim.data = False
        limit_pub.publish(_lim)


    st.error_box = error_box
    st.error_msg = msg
    pub.publish(st)

    time.sleep(0.1)

