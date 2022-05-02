#!/usr/bin/env python3

"""Chopper device simulator.

Original implementation is ogawa-ros/pci_boards_driver/scripts/cpz7415v.py
(https://github.com/ogawa-ros/pci_boards_driver/blob/master/scripts/cpz7415v.py)

"""

import random
import time
import threading  # noqa: F401

from neclib.typing import Literal

import rospy
from std_msgs.msg import Int64

from necst.msg import String_necst


class Chopper:

    # fmt: off
    motion = {
        "x": {
            "clock":  0, "acc_mode": "", "low_speed": 0,
            "speed": 0, "acc": 0, "dec": 0, "step": 0
        },
        "y": {
            "clock":  0, "acc_mode": "", "low_speed": 0,
            "speed": 0, "acc": 0, "dec": 0, "step": 0
        },
        "z": {
            "clock":  0, "acc_mode": "", "low_speed": 0,
            "speed": 0, "acc": 0, "dec": 0, "step": 0
        },
        "u": {
            "clock":  0, "acc_mode": "", "low_speed": 0,
            "speed": 0, "acc": 0, "dec": 0, "step": 0
        }
    }
    # fmt: on

    last_position = {"x": 0, "y": 0, "z": 0, "u": 0}

    def __init__(self):
        self.node_name = "cpz7415v"
        self.rsw_id = 0

        topic_step_u = f"/{self.node_name}_{self.rsw_id + 1}_rsw{self.rsw_id}_u_step"
        topic_step_u_cmd = (
            f"/{self.node_name}_{self.rsw_id + 1}_rsw{self.rsw_id}_u_step_cmd"
        )

        self.pub_step_u = rospy.Publisher(topic_step_u, Int64, queue_size=1)
        self.pub_status_hot = rospy.Publisher("/status_hot", String_necst, queue_size=1)
        rospy.Subscriber(topic_step_u_cmd, Int64, self._step_clbk, callback_args="u")

        self._cmd_recv_timestamp = {"x": 0, "y": 0, "z": 0, "u": 0}
        self._device_response = {"x": 0, "y": 0, "z": 0, "u": 0}

    def _step_clbk(self, msg, axis: Literal["x", "y", "z", "u"]) -> None:
        self.motion[axis]["step"] = msg.data

        self._cmd_recv_timestamp[axis] = time.time()

    def _simulate_response_step(self) -> None:
        now = time.time()
        fluctuation = random.random()
        drive_duration = 2 + fluctuation  # sec

        for axis in "xyzu":
            if self.motion[axis]["step"] != self.last_position[axis]:
                if now - self._cmd_recv_timestamp[axis] > drive_duration:
                    self._device_response[axis] = self.motion[axis]["step"]

    def _get_step(self) -> None:
        step = [self._device_response[axis] for axis in "xyzu"]

        if self.last_position["u"] != step[3]:
            self.pub_step_u.publish(step[3])
            self.pub_status_hot.publish("in" if step[3] == 0 else "out")
            self.last_position["u"] = step[3]

        # Attempt to ensure TCP socket communication to properly be framed.
        time.sleep(0.05)

    def main(self):
        while not rospy.is_shutdown():
            self._simulate_response_step()
            self._get_step()


if __name__ == "__main__":
    rospy.init_node("cpz7415v")
    ctrl = Chopper()
    ctrl.main()
