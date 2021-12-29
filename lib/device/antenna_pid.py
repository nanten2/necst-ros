"""

..Design Policy::

   This script will be executed in high frequency with no vectorization, and there are
   many arrays updated frequently. These mean that the use of Numpy may not be the best
   choice to speed up the calculation. Measure the execution time first, then implement.

"""

import time
from typing import Tuple

import numpy as np

if __name__.endswith(".antenna_pid"):
    from . import utils
else:
    import utils

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
    K_d: float = 0.3

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
        >>> AntennaDevice.with_configuration(pid_param=[2.2, 0, 0])

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
        if getattr(self, "cmd_speed", None) is None:
            utils.update_list(self.cmd_speed, 0)
        utils.update_list(self.time, time.time())
        utils.update_list(self.cmd_coord, cmd_coord)
        utils.update_list(self.enc_coord, enc_coord)
        utils.update_list(self.error, cmd_coord - enc_coord)
        utils.update_list(self.target_speed, 0)

    def initialize(self) -> None:
        if getattr(self, "cmd_speed", None) is None:
            self.cmd_speed = DefaultTwoList.copy()
        self.time = DefaultTwoList.copy() * int(self.ERROR_INTEG_COUNT / 2)
        self.cmd_coord = DefaultTwoList.copy()
        self.enc_coord = DefaultTwoList.copy()
        self.error = DefaultTwoList.copy() * int(self.ERROR_INTEG_COUNT / 2)
        self.target_speed = DefaultTwoList.copy()
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
            If `True`, the telescope won't move regardless of the inputs.
        unit
            Unit in which `cmd_coord` and `enc_coord` are given. One of ["deg",
            "arcmin", "arcsec"]

        Returns
        -------
        speed
            Speed which will be commanded to motor, in original unit.

        """
        # Convert to deg.
        factor = utils.angle_conversion_factor(unit, "deg")
        cmd_coord *= factor
        enc_coord *= factor

        threshold = 100 / 3600  # 100arcsec
        delta_cmd_coord = cmd_coord - self.cmd_coord[Now]
        if np.isnan(self.time[Now]) or (abs(delta_cmd_coord) > threshold):
            self.set_initial_parameters(cmd_coord, enc_coord)
            # Set default values on initial run or on detection of sudden jump of error,
            # which may indicate a change of commanded coordinate.
            # This will give too small `self.dt` later, but that won't propose any
            # problem, since `current_speed` goes to 0, and too large target_speed will
            # be suppressed by speed and acceleration limit.

        current_speed = self.cmd_speed[Now]
        # Encoder readings cannot be used, due to the lack of stability.

        utils.update_list(self.time, time.time())
        utils.update_list(self.cmd_coord, cmd_coord)
        utils.update_list(self.enc_coord, enc_coord)
        utils.update_list(self.error, cmd_coord - enc_coord)
        utils.update_list(
            self.target_speed, (cmd_coord - self.cmd_coord[Now]) / self.dt
        )

        # Calculate and validate drive speed.
        speed = self.calc_pid()
        if abs(self.error[Now]) > (20 / 3600):  # 20arcsec
            # When error is small, smooth control delays the convergence of drive.
            # When error is large, smooth control can avoid overshooting.
            max_diff = utils.clip(self.MAX_ACCELERATION * self.dt, -0.2, 0.2)
            # 0.2 clipping is to avoid large acceleration caused by large dt.
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
        target_acceleration = (
            self.target_speed[Now] - self.target_speed[Last]
        ) / self.dt
        threshold = 2  # 2deg/s^2
        if abs(target_acceleration) > threshold:
            self.target_speed[Now] = 0

        return (
            self.target_speed[Now]
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

        Returns
        -------
        angle
            Unwrapped angle in the same unit as the input.

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
        target_min_candidate = target - 360 * ((target - limits[0]) // 360)
        target_candidates = [
            angle
            for angle in utils.frange(target_min_candidate, limits[1], 360)
            if (limits[0] + margin) < angle < (limits[1] - margin)
        ]
        if len(target_candidates) == 1:
            return target_candidates[0] * utils.angle_conversion_factor("deg", unit)
        else:
            # Avoid over-180deg motion.
            suitable = [
                angle for angle in target_candidates if (angle - current) <= 180
            ][0]
            return suitable * utils.angle_conversion_factor("deg", unit)
