#!/usr/bin/env python3
import math
import time

# import portio
import rospy
import threading
import sys

sys.path.append("../../lib")
import enc

from necst.msg import Status_encoder_msg
from necst.msg import Status_antenna_msg


class enc_controller(object):

    Az = ""
    El = ""
    enc_Az = 0.0  # test
    enc_El = 45 * 3600.0  # test
    resolution = 0.13728813559  # 360*3600/(23600*400) (4 multiplication)

    def __init__(self):
        pass

    """
    def pub_status(self):
        pub = rospy.Publisher("status_encoder", Status_encoder_msg, queue_size = 10, latch = True)
        msg = Status_encoder_msg()

        while not rospy.is_shutdown():
            print("loop...")
            msg.enc_az = self.enc_Az
            msg.enc_el = self.enc_El
            time.sleep(0.1)
            pub.publish(msg)
            rospy.loginfo('Az :'+str(msg.enc_az/3600.))
            rospy.loginfo('El :'+str(msg.enc_el/3600.))
        return
    """

    def counter_thread(self):
        pub = rospy.Publisher(
            "status_encoder", Status_encoder_msg, queue_size=10, latch=True
        )
        msg = Status_encoder_msg()
        while not rospy.is_shutdown():
            pls = enc.enc()
            self.enc_Az = int(pls[0]) * self.resolution
            self.enc_El = int(pls[1]) * self.resolution + 45 * 3600.0
            msg.enc_az = self.enc_Az
            msg.enc_el = self.enc_El
            msg.from_node = "smartcounter_status"
            msg.timestamp = time.time()
            time.sleep(0.05)
            pub.publish(msg)
            rospy.loginfo("Az :" + str(msg.enc_az / 3600.0))
            rospy.loginfo("El :" + str(msg.enc_el / 3600.0))


if __name__ == "__main__":
    rospy.init_node("smartcounter_status")
    encoder = enc_controller()
    encoder.counter_thread()
    rospy.spin()
