"""

..Design Policy::

   This script will be executed in high frequency with no vectorization, and there are
   many arrays updated frequently. These mean that the use of Numpy may not be the best
   choice to speed up the calculation. Measure the execution time first, then implement.

"""

import time
from typing import Tuple

import numpy as np

from . import utils

# Indices for 2-lists (mutable version of so-called 2-tuple).
Last = -2
Now = -1
# Default value for 2-lists.
DefaultTwoList = [np.nan, np.nan]


class PIDController:
    """Controller of telescope antenna drive.

    Notes
    -----
    - Both the NANTEN2 and 1.85-m telescopes uses servomotors for antenna drive.

    """

    K_p: float = 1.0
    K_i: float = 0.5
    K_p: float = 0.3

    MAX_SPEED: float = 2  # deg/s
    MAX_ACCELERATION: float = 2  # deg/s^2

    ERROR_INTEG_COUNT: int = 50  # Keep last 50 data for error integration.
    # Time interval of error integral varies according to PID calculation frequency,
    # which may cause optimal PID parameters to change according to the frequency.

    def __init__(self) -> None:
        # Initialize parameters.
        self.initialize()

    @classmethod
    def with_configuration(
        cls,
        *,
        pid_param: Tuple[float, float, float] = None,
        max_speed: float = None,
        max_acceleration: float = None,
        error_integ_count: int = None,
    ) -> "PIDController":
        """Initialize `AntennaDevice` class with properly configured parameters.

        Examples
        --------
        >>> AntennaDevice.with_configuration("az", pid_param=[2.2, 0, 0])

        """
        if pid_param is not None:
            cls.K_p, cls.K_i, cls.K_d = pid_param
        if max_speed is not None:
            cls.MAX_SPEED = max_speed
        if max_acceleration is not None:
            cls.MAX_ACCELERATION = max_acceleration
        if error_integ_count is not None:
            cls.ERROR_INTEG_COUNT = error_integ_count
        return cls()

    @property
    def dt(self) -> float:
        """Time interval of last 2 PID calculations."""
        return self.time[Now] - self.time[Last]

    @property
    def error_integral(self) -> float:
        _time, _error = np.array(self.time), np.array(self.error)
        dt = _time[1:] - _time[:-1]
        error_interpolated = (_error[1:] + _error[:-1]) / 2
        return np.nansum(error_interpolated * dt)

    @property
    def error_derivative(self) -> float:
        return (self.error[Now] - self.error[Last]) / self.dt

    def set_initial_parameters(self, cmd_coord: float, enc_coord: float) -> None:
        self.initialize()
        utils.update_list(self.cmd_speed, 0)
        utils.update_list(self.time, time.time())
        utils.update_list(self.cmd_coord, cmd_coord)
        utils.update_list(self.enc_coord, enc_coord)
        utils.update_list(self.error, cmd_coord - enc_coord)

    def initialize(self) -> None:
        self.cmd_speed = DefaultTwoList.copy()
        self.time = DefaultTwoList.copy() * int(self.ERROR_INTEG_COUNT / 2)
        self.cmd_coord = DefaultTwoList.copy()
        self.enc_coord = DefaultTwoList.copy()
        self.error = DefaultTwoList.copy() * int(self.ERROR_INTEG_COUNT / 2)
        # Without `copy()`, updating one of them updates all its shared (not copied)
        # objects.

    def get_speed(
        self,
        cmd_coord: float,
        enc_coord: float,
        stop: bool = False,
        unit: str = "deg",
    ) -> float:
        """Calculates valid drive speed.

        Parameters
        ----------
        cmd_coord
            Instructed AzEl coordinate.
        enc_coord
            AzEl encoder reading.
        stop
            If `True`, the telescope won't move.
        unit
            Unit in which `cmd_coord` and `enc_coord` are given. One of ["deg",
            "arcmin", "arcsec"]

        Returns
        -------
        float
            Speed which is commanded to motor, in [deg/s].

        """
        # Convert to deg.
        factor = utils.angle_conversion_factor(unit, "deg")
        cmd_coord *= factor
        enc_coord *= factor

        threshold = 2.5 / self.dt  # 2.5deg/s
        if np.isnan(self.time[Now]) or (abs(self.error_derivative) > threshold):
            self.set_initial_parameters(cmd_coord, enc_coord)
            # Set default values on initial run or on detection of sudden jump of error,
            # which may indicate a change of commanded coordinate.
            # This will give too small `self.dt` later, but that won't propose any
            # problem, since `current_speed` goes to 0, and too large D-term will be
            # suppressed by speed and acceleration limit.

        current_speed = self.cmd_speed[Now]
        # Encoder readings cannot be used, due to the lack of stability.

        utils.update_list(self.time, time.time())
        utils.update_list(self.cmd_coord, cmd_coord)
        utils.update_list(self.enc_coord, enc_coord)
        utils.update_list(self.error, cmd_coord - enc_coord)

        # Calculate and validate drive speed.
        speed = self.calc_pid()
        if self.error[Now] > 0.1:  # 0.1deg
            # When error is small, smooth control delays the convergence of drive.
            # When error is large, smooth control can avoid overshooting.
            max_diff = self.MAX_ACCELERATION * self.dt
            speed = utils.clip(
                speed, current_speed - max_diff, current_speed + max_diff
            )  # Limit acceleration.
        speed = utils.clip(speed, -1 * self.MAX_SPEED, self.MAX_SPEED)  # Limit speed.

        if stop:
            utils.update_list(self.cmd_speed, 0)
        else:
            utils.update_list(self.cmd_speed, speed)

        return self.cmd_speed[Now] * utils.angle_conversion_factor("deg", unit)

    def calc_pid(self) -> float:
        # Speed of the move of commanded coordinate. This includes sidereal motion, scan
        # speed, and other non-static component of commanded value.
        target_speed = (self.cmd_coord[Now] - self.cmd_coord[Last]) / self.dt

        return (
            target_speed
            + self.K_p * self.error[Now]
            + self.K_i * self.error_integral
            + self.K_d * self.error_derivative
        )

    @staticmethod
    def suitable_angle(
        current: float,
        target: float,
        limits: Tuple[float, float],
        margin: float = 40,
        unit: str = "deg",
    ) -> float:
        """Find suitable unwrapped angle.

        Notes
        -----
        Azimuthal control of telescope should avoid
        1. 360deg motion during observation. This mean you should observe around
        -100deg, not 260deg, to command telescope of [-270, 270]deg limit.
        2. Over-180deg motion. Both 170deg and -190deg are safe in avoiding the 360deg
        motion, but if the telescope is currently directed at 10deg, you should select
        170deg to save time.

        """
        assert limits[0] < limits[1], "Limits should be given in ascending order."
        factor = utils.angle_conversion_factor(unit, "deg")
        current *= factor
        target *= factor
        limits = [lim * factor for lim in limits]
        margin *= factor

        # Avoid 360deg motion.
        safety_margin = margin  # deg
        target_min_candidate = target - 360 * ((target - limits[0]) // 360)
        target_candidates = [
            angle + (target_min_candidate % 1)
            for angle in range(int(target_min_candidate), int(limits[1]), 360)
            if (limits[0] + safety_margin) < angle < (limits[1] - safety_margin)
        ]
        if len(target_candidates) == 1:
            return target_candidates[0] * utils.angle_conversion_factor("deg", unit)
        else:
            # Avoid over-180deg motion.
            suitable = [
                angle for angle in target_candidates if (angle - current) <= 180
            ][0]
            return suitable * utils.angle_conversion_factor("deg", unit)
