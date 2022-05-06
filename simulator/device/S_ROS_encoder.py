#!/usr/bin/env python3

import sys
import time

from neclib.simulators import AntennaEncoderEmulator
from neclib.utils import AzElData

import rospy

from necst.msg import Status_antenna_msg, Status_encoder_msg

sys.path.append("/home/amigos/ros/src/necst/scripts/device")
import ROS_encoder  # noqa: E402


class EncoderControllerSimulator(ROS_encoder.enc_controller):

    node_name = "encoder_status"

    def __init__(self) -> None:
        self.speed = AzElData(0, 0)

        AntennaEncoderEmulator.ANGLE_UNIT = "arcsec"
        self.encoder = AntennaEncoderEmulator()

        rospy.Subscriber(
            "/status_antenna", Status_antenna_msg, self._pid_clbk, queue_size=1
        )
        self.PUB_status = rospy.Publisher(
            "status_encoder", Status_encoder_msg, queue_size=1, latch=True
        )

    def _pid_clbk(self, msg: Status_antenna_msg) -> None:
        self.speed.az = msg.command_azspeed
        self.speed.el = msg.command_elspeed

    def pub_status(self) -> None:
        self.rate = rospy.Rate(100)  # Hz

        while not rospy.is_shutdown():
            encoder_reading = self.encoder.read()

            msg = Status_encoder_msg()
            msg.enc_az = encoder_reading.az
            msg.enc_el = encoder_reading.el
            msg.from_node = self.node_name
            msg.timestamp = time.time()
            self.PUB_status.publish(msg)

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node(EncoderControllerSimulator.node_name)
    enc = EncoderControllerSimulator()
    enc.pub_status()
