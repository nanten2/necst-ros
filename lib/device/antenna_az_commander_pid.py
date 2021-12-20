#!/usr/bin/env python3

from typing import Tuple

import rospy
from std_msgs.msg import Float64

from . import utils
from .antenna_pid import PIDController

node_name = "antenna_az_commander_pid"

# Indices for 2-lists (mutable version of so-called 2-tuple).
Last = -2
Now = -1


class antenna_az_feedback:
    """For 1.85-m telescope."""

    speed_d = 0
    pre_deg = 0
    pre_hensa = 0
    enc_before = 0
    ihensa = 0
    i_ave_num = 10
    t_now = t_past = 0
    current_speed = 0
    deg_enc = 0
    lock = False

    SPEED2RATE: float

    def __init__(self):
        self.gear_ratio = rospy.get_param("~gear_ratio")
        self.pulseper360deg = rospy.get_param("~pulseper360deg")
        self.pulse_a = rospy.get_param("~pulse_a")
        self.pulse_b = rospy.get_param("~pulse_b")
        self.SPEED2RATE = (
            (self.gear_ratio / 360)
            * self.pulseper360deg
            * (self.pulse_b / self.pulse_a)
        )  # Speed [deg/s] to servomotor rate.

        self.p_coeff = rospy.get_param("~p_coeff")
        self.i_coeff = rospy.get_param("~i_coeff")
        self.d_coeff = rospy.get_param("~d_coeff")
        # Following 2 are (derivative of) rate, not speed.
        self.MOTOR_MAX_STEP = rospy.get_param("~MOTOR_MAXSTEP")
        self.MOTOR_AZ_MAXSPEED = rospy.get_param("~MOTOR_AZ_MAXSPEED")
        self.controller = PIDController.with_configuration(
            pid_param=[self.p_coeff, self.i_coeff, self.d_coeff],
            max_speed=self.MOTOR_AZ_MAXSPEED / self.SPEED2RATE,
            max_acceleration=2,
            error_integ_count=self.i_ave_num,
        )

        self.hensa_stock = self.controller.error  # Alias.

        self.init_ros()
        self.topic_to = self.Publisher["calculated_rate"]
        self.topic_cur = self.Publisher["current_speed"]
        self.topic_tar = self.Publisher["target_speed"]
        self.topic_hensa = self.Publisher["error"]

    def init_ros(self):
        _publisher_conf = {
            "calculated_rate": ["/1p85m/az_speed", Float64, 1],
            "current_speed": ["/1p85m/az_current_speed", Float64, 1],
            "target_speed": ["/1p85m/az_target_speed", Float64, 1],
            "error": ["/1p85m/az_pid_hensa", Float64, 1],
        }
        _subscriber_conf = {
            "commanded_coord": ["/1p85m/az_cmd2", Float64, self.antenna_az_feedback, 1],
            "encoder_coord": ["/1p85m/az", Float64, self.antenna_az_encoder, 1],
        }
        self.Publisher = {
            key: rospy.Publisher(name, msg_type, queue_size=n)
            for key, (name, msg_type, n) in _publisher_conf.items()
        }
        self.Subscriber = {
            key: rospy.Subscriber(name, msg_type, callback, queue_size=n)
            for key, (name, msg_type, callback, n) in _subscriber_conf.items()
        }

    def antenna_az_feedback(self, command: Float64):
        speed = self.controller.get_speed(command.data, self.enc_coord, unit="deg")
        self.current_speed = self.controller.cmd_speed[Now]
        speed = utils.clip(
            speed,
            self.current_speed - self.MOTOR_MAX_STEP,
            self.current_speed + self.MOTOR_MAX_STEP,
        )
        rate = speed * self.SPEED2RATE
        if self.lock:
            rate = 0
        self.speed_d = rate
        self.Publisher["calculated_rate"].publish(rate)
        target_speed = (
            self.controller.cmd_coord[Now] - self.controller.cmd_coord[Last]
        ) / self.controller.dt
        self.Publisher["target_speed"].publish(target_speed)
        self.Publisher["current_speed"].publish(self.current_speed)
        self.Publisher["error"].publish(self.controller.error[Now])

        self.t_past = self.controller.time[Last]
        self.t_now = self.controller.time[Now]
        self.pre_hensa = self.controller.error[Last]
        self.pre_deg = self.controller.cmd_coord[Last]
        self.enc_before = self.controller.enc_coord[Last]
        self.ihensa = self.controller.error_integral

    def antenna_az_encoder(self, status: Float64):
        self.enc_coord = status.data
        self.deg_enc = self.enc_coord  # Alias.

    def antenna_az_pid(self, status: Float64):
        """No topic triggers this callback function."""
        self.p_coeff = self.controller.K_p = status.data[0]
        self.i_coeff = self.controller.K_i = status.data[1]
        self.d_coeff = self.controller.K_d = status.data[2]

    def calc_pid(
        self,
        target_deg: float,
        encoder_deg: float,
        pre_deg: float,
        pre_hensa: float,
        ihensa: float,
        enc_before: float,
        t_now: float,
        t_past: float,
        p_coeff: float,
        i_coeff: float,
        d_coeff: float,
    ) -> Tuple[float, float]:
        """PID calculation.

        ..deprecated:: v3.1.0
            This function will be removed in v4.0.0, because of implementation structure
            issue. Please use `AntennaDevice.calc_pid`.

        """
        calculator = PIDController.with_configuration(
            pid_param=[p_coeff, i_coeff, d_coeff]
        )

        # Set `Last` parameters.
        calculator._update(calculator.time, t_past)
        calculator._update(calculator.cmd_coord, pre_deg)
        calculator._update(calculator.enc_coord, enc_before)
        calculator._update(calculator.error, pre_hensa)

        # Set `Now` parameters.
        calculator._update(calculator.time, t_now)
        calculator._update(calculator.cmd_coord, target_deg)
        calculator._update(calculator.enc_coord, encoder_deg)
        calculator._update(calculator.error, target_deg - encoder_deg)
        calculator._update(calculator.error_integ, ihensa)

        speed = calculator.calc_pid()

        self.hensa_stock = calculator.error
        self.current_speed = calculator.cmd_speed[Now]

        target_speed = (
            self.controller.cmd_coord[Now] - self.controller.cmd_coord[Last]
        ) / self.controller.dt
        self.Publisher["target_speed"].publish(target_speed)
        self.Publisher["current_speed"].publish(self.current_speed)
        self.Publisher["error"].publish(self.controller.error[Now])

        return (speed, calculator.error_integ)


if __name__ == "__main__":
    rospy.init_node(node_name)
    antenna_az_feedback()
    rospy.spin()
