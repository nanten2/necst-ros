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
        rospy.init_node(self.node_name)
        self.speed = AzElData(0, 0)

        AntennaEncoderEmulator.ANGLE_UNIT = "arcsec"
        self.encoder = AntennaEncoderEmulator()
        self.encoder.position.az = 0

        rospy.Subscriber(
            "/status_antenna", Status_antenna_msg, self._pid_clbk, queue_size=1
        )
        self.publisher = {
            "status": rospy.Publisher(
                "status_encoder", Status_encoder_msg, queue_size=1, latch=True
            )
        }

    def _pid_clbk(self, msg: Status_antenna_msg) -> None:
        self.speed.az = msg.command_azspeed
        self.speed.el = msg.command_elspeed

        self.encoder.command(self.speed.az, "az")
        self.encoder.command(self.speed.el, "el")
        # TODO: Check sudden jump of encoder reading when it nears the command value

    def pub_status(self) -> None:
        self.rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            encoder_reading = self.encoder.read()

            msg = Status_encoder_msg(
                enc_az=encoder_reading.az,
                enc_el=encoder_reading.el,
                from_node=self.node_name,
                timestamp=time.time(),
            )
            self.publisher["status"].publish(msg)

            self.rate.sleep()


if __name__ == "__main__":
    enc = EncoderControllerSimulator()
    enc.pub_status()
