#!/usr/bin/env python3

import time
from typing import Any, Dict, Tuple

import pyinterface

# Indeces for 2-lists.
Last = 0
Now = 1


class AntennaDevice:
    """Controller of telescope antenna drive.

    Parameters
    ----------
    azel
        "Az" or "el", case insensitive.
    board_name
    rsw_id

    Notes
    -----
    - The NANTEN2 telescope is powered by servomotors.
    - This class should be integrated with ROS, instead of returning numerous parameters
    in `pid` method.
    """

    K_p: float = 2.2
    K_i: float = 0
    K_d: float = 0

    MAX_SPEED: int = 2  # deg/s
    MAX_ACCELERATION: int = 2  # deg/s^2

    SPEED2RATE = (7 / 12) * 10000  # Unit of speed is deg/s. Ref. N2-7395

    def __init__(self, azel: str, board_name: int = 2724, rsw_id: int = 0) -> None:
        self.driver = AntennaDriver(board_name, rsw_id)
        self.azel = azel

        self.cmd_speed = [None, None]
        self.time = [None, None]
        self.cmd_coord = [None, None]
        self.enc_coord = [None, None]
        self.error = [None, None]
        self.error_integ = [None, None]
        # `error_integ` has no need to be keep `Last` value. Just compatibility issue
        # with other parameters.

    @classmethod
    def with_configuration(
        cls,
        azel: str,
        *,  # Positional arguments are not allowed hereafter.
        pid_param: Tuple[float, float, float] = None,
        max_speed: int = None,
        max_acceleration: int = None,
        **kwargs,
    ) -> "AntennaDevice":
        """Initialize `AntennaDevice` class with properly configured parameters.

        Examples
        --------
        >>> AntennaDevice.with_configuration("az", pid_param=[2.2, 0, 0])
        """
        inst = cls(azel, **kwargs)
        if pid_param is not None:
            inst.K_p, inst.K_i, inst.K_d = pid_param
        if max_speed is not None:
            inst.MAX_SPEED = max_speed
        if max_acceleration is not None:
            inst.MAX_ACCELERATION = max_acceleration
        return inst

    def init_speed(self) -> None:
        self.command(0)

    def command(self, rate: int) -> None:
        """
        Parameters
        ----------
        rate
            Command to servo motor, which follows the formula
            $command[rpm] = 1500rpm * (rate / 100)%$ hence $0 <= rate <= 10000$. 1500rpm
            is the max speed of the motor installed on the NANTEN2.
        """
        self.driver.command(int(rate), self.azel)

    def _update(self, param_name: str, new_value: Any) -> None:
        parameter = getattr(self, param_name)
        parameter = parameter[1:]
        parameter.append(new_value)
        setattr(self, param_name, parameter)

    @staticmethod
    def _clip(value: float, minimum: float, maximum: float) -> float:
        return min(max(minimum, value), maximum)

    @property
    def dt(self) -> float:
        return self.time[Now] - self.time[Last]

    def initialize(self, cmd_coord: float, enc_coord: float) -> None:
        self._update("cmd_speed", 0)
        self._update("time", time.time())
        self._update("cmd_coord", cmd_coord)
        self._update("enc_coord", enc_coord)
        self._update("error", 0)
        self._update("error_integ", 0)

    def drive(
        self,
        cmd_coord: float,
        enc_coord: float,
        stop: bool = False,
        unit: str = "arcsec",
    ) -> Dict[str, float]:
        """
        Parameters
        ----------
        cmd_coord
            In arcsec.
        enc_coord
            In arcsec.
        stop
        """
        if unit.lower() == "arcsec":
            # Convert to deg.
            cmd_coord /= 3600
            enc_coord /= 3600
        elif unit.lower() != "deg":
            raise ValueError("Unit other than 'deg' or 'arcsec' isn't supported.")

        if self.time[Now] is None:
            self.initialize(cmd_coord, enc_coord)  # Set default values.
            # This will give too small `self.dt` later, but that won't propose any
            # problem, since `current_speed` goes to 0, and too large D-term 1) will be
            # ignored in `self.calc_pid` and also 2) the contribution from that term
            # will be suppressed by speed and acceleration limit.

        # Avoid over-180deg drive for Az control. For valid El control, the conditions
        # will never be satisfied, so this functionality is safely placed here without
        # any conditional context like `if azel == "az":`.
        # The value range of `enc_coord` is -270deg ~ 0deg ~ +270deg. The origin of the
        # following magic number 40 is unknown.
        magic = 40
        if (enc_coord > magic) and (cmd_coord + 360 < (180 + magic)):
            cmd_coord += 360
        elif (enc_coord < -1 * magic) and (cmd_coord - 360 > -1 * (180 + magic)):
            cmd_coord -= 360

        self._update("time", time.time())
        self._update("cmd_coord", cmd_coord)
        self._update("enc_coord", enc_coord)
        self._update("error", cmd_coord - enc_coord)
        self._update("error_integ", self.error_integ[Now] + self.error[Now] * self.dt)

        # Calculate and validate drive speed.
        speed = self.calc_pid()
        self._update("cmd_speed", speed)
        speed = self._clip(
            speed, -1 * self.MAX_SPEED, self.MAX_SPEED
        )  # Limit the speed.
        max_diff = self.MAX_ACCELERATION * self.dt
        current_speed = (self.enc_coord[Last] - self.enc_coord[Now]) / self.dt
        speed = self._clip(
            speed, current_speed - max_diff, current_speed + max_diff
        )  # Limit the acceleration.
        self.cmd_speed[Now] = speed

        if stop:
            self.command(0)
        else:
            self.command(int(self.cmd_speed[Now] * self.SPEED2RATE))

        return {
            "speed": self.cmd_speed[Now] * 3600,
            "p_term": self.K_p * self.error[Now] * 3600,
            "i_term": self.K_i * self.error_integ[Now] * 3600 * self.dt,
            "d_term": self.K_d * (self.error[Now] - self.error[Last]) * 3600 / self.dt,
            "cmd_coord": self.cmd_coord[Now] * 3600,
            "enc_coord": self.enc_coord[Now] * 3600,
            "error_last": self.error[Last] * 3600,
            "error_integ": self.error_integ[Now] * 3600,
            "enc_coord_last": self.enc_coord[Last] * 3600,
            "t_now": self.time[Now],
            "t_last": self.time[Last],
        }  # All angle-related parameters are in arcsec.

    def calc_pid(self) -> float:
        error_derivative = (self.error[Now] - self.error[Last]) / self.dt

        # Speed of the move of commanded coordinate. This includes sidereal motion, scan
        # speed, and other non-static component of commanded value.
        target_speed = (self.cmd_coord[Now] - self.cmd_coord[Last]) / self.dt

        # When commanded coordinate is too far from current position, reset integration
        # of the error.
        threshold = 50  # arcsec
        if abs(self.error[Now]) > threshold / 3600:
            self.error_integ[Now] = 0
        # When time derivative of the error is too large, ignore the D-term.
        threshold = 1.5 / self.dt  # 1.5deg/s
        if abs(error_derivative) > threshold:
            error_derivative = 0

        speed = (
            target_speed
            + self.K_p * self.error[Now]
            + self.K_i * self.error_integ[Now]
            + self.K_d * error_derivative
        )
        return speed

    def emergency_stop(self) -> None:
        for _ in range(5):
            self.command(0)
            time.sleep(0.05)
            self._update("cmd_speed", 0)


class AntennaDriver:
    def __init__(self, board_name: int, rsw_id: int) -> None:
        """
        Parameters
        ----------
        board_name
            Name? of Interface board.
        rsw_id
            Rotary SWitch ID.
        """
        self.dio = pyinterface.open(board_name, rsw_id)
        self.dio.initialize()

    def command(self, value: int, azel: str) -> None:
        # Specify which pins to use.
        if azel.lower() == "az":
            target = "OUT1_16"
        elif azel.lower() == "el":
            target = "OUT17_32"

        cmd = bin(value)[2:].zfill(16)[::-1]  # [::-1] for little endian.
        cmd = [int(char) for char in cmd]
        self.dio.output_word(target, cmd)


class antenna_device:
    """Alias of `AntennaDevice`, for backward compatibility."""

    command_az_speed = command_el_speed = 0
    az_rate_d = el_rate_d = 0
    pre_hensa = [0, 0]
    ihensa = [0, 0]
    enc_before = [0, 0]
    pre_arcsec = [0, 0]
    t_now = t_past = 0
    p_coeff = [2.2, 2.2]
    i_coeff = [0, 0]
    d_coeff = [0, 0]
    dir_name = ""

    def __init__(self) -> None:
        self._az = AntennaDevice("az")
        self._el = AntennaDevice("el")
        self.dio = self._az.device.dio  # No difference if `self._el` is used.

    def init_speed(self) -> None:
        self._az.init_speed()
        self._el.init_speed()

    def set_pid_param(self, param: Dict[str, Tuple[float, float, float]]) -> None:
        self._az.K_p, self._az.K_i, self._az.K_d = param["az"]
        self._el.K_p, self._el.K_i, self._el.K_d = param["el"]

    def move_azel(
        self,
        az_arcsec: float,
        el_arcsec: float,
        enc_az: float,
        enc_el: float,
        pid_param: Dict[str, Tuple[float, float, float]] = None,
        m_bStop: str = "FALSE",
    ) -> Tuple[float, ...]:
        if pid_param is not None:
            self.set_pid_param(pid_param)

        if m_bStop == "False":
            stop = False
        elif m_bStop == "TRUE":
            stop = True

        self.t_past = self._az.time[Last]  # Overwritten soon after.
        # May slightly different from `self._el.time[Last]`.

        ret_az = self._az.drive(az_arcsec, enc_az, stop=stop, unit="arcsec")
        ret_el = self._el.drive(el_arcsec, enc_el, stop=stop, unit="arcsec")

        self.t_now = self._az.time[Now]
        # May slightly different from `self._el.time[Now]`.

        self.enc_before = [self._az.enc_coord[Last], self._el.enc_coord[Last]]
        self.pre_hensa = [self._az.error[Last], self._el.error[Last]]
        self.pre_arcsec = [self._az.cmd_coord[Last], self._el.cmd_coord[Last]]
        self.ihensa = [self._az.error_integ[Now], self._el.error_integ[Now]]
        self.t_past = self.t_now

        self.az_rate_d = ret_az["speed"]
        self.el_rate_d = ret_el["speed"]

        self.command_az_speed = int(self.az_rate_d)
        self.az_rate_d = self.command_az_speed
        self.command_el_speed = int(self.el_rate_d)
        self.el_rate_d = self.command_el_speed  # Cannot understand what's done here.

        return list(ret_az.values())[:-2] + list(ret_el.values())

    def emergency_stop(self) -> None:
        self._az.emergency_stop()
        self._el.emergency_stop()


def calc_pid(
    target_arcsec: float,
    encoder_arcsec: float,
    pre_arcsec: float,
    pre_hensa: float,
    ihensa: float,
    enc_before: float,
    t_now: float,
    t_past: float,
    p_coeff: float,
    i_coeff: float,
    d_coeff: float,
) -> Tuple[float, ...]:
    calculator = AntennaDevice.with_configuration(
        "az", pid_param=[p_coeff, i_coeff, d_coeff]
    )  # No difference if `"el"` is passed.

    # Set `Last` parameters.
    calculator._update("time", t_past)
    calculator._update("cmd_coord", pre_arcsec / 3600)
    calculator._update("enc_coord", enc_before / 3600)
    calculator._update("error", pre_hensa / 3600)

    # Set `Now` parameters.
    calculator._update("time", t_now)
    calculator._update("cmd_coord", target_arcsec / 3600)
    calculator._update("enc_coord", encoder_arcsec / 3600)
    calculator._update("error", (target_arcsec - encoder_arcsec) / 3600)
    calculator._update("error_integ", ihensa / 3600)

    speed = calculator.calc_pid()

    error_diff = (calculator.error[Now] - calculator.error[Last]) / calculator.dt
    if abs(error_diff) * 3600 > 1:
        error_diff = 0

    return [
        speed,
        calculator.error_integ[Now],
        calculator.K_p * calculator.error[Now],
        calculator.K_i * calculator.error_integ[Now] * calculator.dt,
        calculator.K_d * error_diff / calculator.dt,
    ]
