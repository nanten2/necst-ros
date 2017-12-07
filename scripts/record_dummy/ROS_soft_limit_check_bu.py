#!/usr/bin/env python                                                           
import time
import rospy
import threading
from necst.msg import Status_encoder_msg
from std_msgs.msg import String

class soft_limit_check(object):
    enc_az = 0.
    enc_el = 45.
    def __init__(self):
        self.pub = rospy.Publisher('move_stop', String, queue_size = 1, latch = True)
        self.call = String()
        self.call.data = 'True'
        pass
    
    def set_enc_parameter(self, req):
        self.enc_az = req.enc_az
        self.enc_el = req.enc_el
        return

    def limit_check(self):
        while True:
            if self.enc_az >= 280*3600 or self.enc_az <=-280*3600:
                rospy.logwarn('[ROS.soft_limit_check.py] : !!!limit az!!!')
                self.pub.publish(self.call.data)
            if self.enc_el >= 90*3600 or self.enc_el <=-1*3600:#?
                rospy.logwarn('[ROS.soft_limit_check.py] : !!!limit el!!!')
                self.pub.publish(self.call.data)
            time.sleep(0.1)
            continue


if __name__ == '__main__':
    print('[ROS_soft_limit_check.py] : Subscribe Start')
    s_check = soft_limit_check()
    th = threading.Thread(target = s_check.limit_check)
    th.setDaemon(True)
    th.start()
    rospy.init_node('Status')
    sub = rospy.Subscriber('status_encoder', Status_encoder_msg, s_check.set_enc_parameter)
    rospy.spin()
