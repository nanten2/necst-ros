import time
import pyinterface
import rospy
from necst.msg import Status_limit_msg

self.dio = pyinterface.create_gpg2000(3)

stop_flag = 0
ret = [1]*4
error_box = [1]*32
msg = ""

rospy.init_node("limit_check")
pub = rospy.Publisher("limit_check", Status_limit_msg, queue_size=10, latch=True)
st = Status_limit_msg()
limit_pub = rospy.Publisher("limit", Bool, queue_size=10, latch=True)
_lim = Bool()

while not rospy.is_shutdown():

    ret[0] = self.dio.ctrl.in_byte('FBIDIO_IN1_8')
    ret[1] = self.dio.ctrl.in_byte('FBIDIO_IN9_16')
    ret[2] = self.dio.ctrl.in_byte('FBIDIO_IN17_24')
    ret[3] = self.dio.ctrl.in_byte('FBIDIO_IN25_32')
    
    for i in range(8):
        error_box[i] = ret[0] >> i & 0x01
    for i in range(8):
        error_box[i+8] = ret[1] >> i & 0x01
    for i in range(8):
        error_box[i+16] = ret[2] >> i & 0x01
    for i in range(8):
        error_box[i+24] = ret[3] >> i & 0x01
        
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
    if stop_flag == 1:
        _lim.msg = False
        limit_pub.publih(_lim)


    st.error_box = error_box
    st.error_msg = msg
    pub.publish(st)

    time.slep(0.1)

