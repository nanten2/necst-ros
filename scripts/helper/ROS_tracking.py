#!/usr/bin/env python3

import time
from concurrent.futures import ThreadPoolExecutor

from neclib import utils
from neclib.utils import AzElData

import rospy

from necst.msg import Bool_necst, Move_mode_msg, Status_antenna_msg, Status_encoder_msg


class TrackingCheck:
    """Ensure tracking accuracy.

    .. note::

       Reason for suspension of checking (subscribing /onepoint_command,
       /linear_command and /planet_command) and interaction with /move_stop topic are
       currently unknown.

    """

    node_name = "tracking"
    ANGLE_UNIT = "arcsec"

    def __init__(self) -> None:
        rospy.init_node(self.node_name)

        rospy.Subscriber("/status_antenna", Status_antenna_msg, self.command_clbk)
        rospy.Subscriber("/status_encoder", Status_encoder_msg, self.encoder_clbk)
        rospy.Subscriber(
            "/onepoint_command", Move_mode_msg, self.onepoint_cmd_clbk, queue_size=1
        )
        rospy.Subscriber(
            "/linear_command", Move_mode_msg, self.linear_cmd_clbk, queue_size=1
        )
        rospy.Subscriber(
            "/planet_command", Move_mode_msg, self.planet_cmd_clbk, queue_size=1
        )

        self.publisher = {
            "tracking_status": rospy.Publisher(
                "/tracking_check", Bool_necst, queue_size=1
            ),
            "move_stop": rospy.Publisher("/move_stop", Bool_necst, queue_size=1),
        }

        self.pid_command = AzElData()
        self.encoder_reading = AzElData()

        self.tracking_check_disabled = False
        """If True, temporarily disable tracking check."""
        self.tracking_check_disable_duration = 5  # sec
        """Duration of temporal stop of tracking check."""
        self.tracking_ok = False
        """If True, tracking error is smaller than threshold."""

    def command_clbk(self, msg: Status_antenna_msg) -> None:
        self.pid_command.az = msg.command_az
        self.pid_command.el = msg.command_el

    def encoder_clbk(self, msg: Status_encoder_msg) -> None:
        self.encoder_reading.az = msg.enc_az
        self.encoder_reading.el = msg.enc_el

    def onepoint_cmd_clbk(self, msg: Move_mode_msg) -> None:
        # TODO: Infer the intention of this implementation.
        self.command = msg
        self.tracking_check_disabled = True

    def linear_cmd_clbk(self, msg: Move_mode_msg) -> None:
        # TODO: Infer the intention of this implementation.
        self.tracking_check_disabled = True

    def planet_cmd_clbk(self, msg: Move_mode_msg) -> None:
        # TODO: Infer the intention of this implementation.
        self.tracking_check_disabled = True

    def _approx_separation(self, error_x: float, error_y: float) -> float:
        return (error_x ** 2 + error_y ** 2) ** 0.5

    def tracking_check(self) -> None:
        _360deg = 360 * utils.angle_conversion_factor("deg", self.ANGLE_UNIT)
        tracking_ok_start = None  # Time current streak of passing checks started.

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.tracking_check_disabled:
                self.tracking_ok = False
                time.sleep(self.tracking_check_disable_duration)
                self.tracking_check_disabled = False

            error_az = abs(self.pid_command.az - self.encoder_reading.az)
            error_el = abs(self.pid_command.az - self.encoder_reading.az)

            dx = utils.dAz2dx(error_az, self.encoder_reading.el, self.ANGLE_UNIT)
            approx_separation = self._approx_separation(
                dx % _360deg, error_el % _360deg
            )

            _3arcsec = 3 * utils.angle_conversion_factor("arcsec", self.ANGLE_UNIT)
            if approx_separation < _3arcsec:
                if tracking_ok_start is None:
                    tracking_ok_start = time.time()
                    self.tracking_ok = False
                if time.time() - tracking_ok_start > 0.5:
                    self.tracking_ok = True
            else:
                tracking_ok_start = None
                self.tracking_ok = False

            rate.sleep()

    def publish_tracking_status(self) -> None:
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            msg = Bool_necst(
                data=self.tracking_ok,
                from_node=self.node_name,
                timestamp=time.time(),
            )
            self.publisher["tracking_status"].publish(msg)

            rate.sleep()

    def publish_movestop_flag(self) -> None:
        # TODO: Infer the intention of this implementation.
        rate = rospy.Rate(2)
        _flag = 0  # Unknown parameter.
        while not rospy.is_shutdown():
            if not hasattr(self, "command"):
                rate.sleep()
                continue

            timestamp = self.command.timestamp
            if not _flag == timestamp:
                if not self.command.coord.lower() == "altaz":
                    _flag = timestamp
                    continue
                # Wait for antenna drive.
                time.sleep(self.tracking_check_disable_duration)
                if self.tracking_ok:
                    msg = Bool_necst(
                        data=False,
                        from_node=__file__,  # TODO: Infer reason not being node name.
                        timestamp=time.time(),
                    )
                    self.publisher["move_stop"].publish(msg)
                    _flag = timestamp
            rate.sleep()

    def start_thread(self) -> None:
        with ThreadPoolExecutor() as executor:
            threads = [
                self.tracking_check,
                self.publish_tracking_status,
                self.publish_movestop_flag,
            ]
            _ = [executor.submit(target) for target in threads]


if __name__ == "__main__":
    node = TrackingCheck()
    node.start_thread()
