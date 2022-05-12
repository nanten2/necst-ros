import time
from typing import Dict, Tuple

try:
    from typing import Literal
except ImportError:
    from typing_extensions import Literal

from neclib.controllers import PIDController
from neclib.utils import optimum_angle

# Indices for parameter lists.
Last = -2
Now = -1


class AntennaDriver:
    """Hardware command layer.

    Parameters
    ----------
    board_model
        Model of Interface DIO board.
    rsw_id
        Rotary SWitch ID.

    """

    def __init__(self, board_model: int, rsw_id: int) -> None:
        import pyinterface

        self.dio = pyinterface.open(board_model, rsw_id)
        self.dio.initialize()

    def command(self, value: int, azel: Literal["az", "el"]) -> None:
        # Specify which pins to use.
        if azel.lower() == "az":
            target = "OUT1_16"
        elif azel.lower() == "el":
            target = "OUT17_32"

        n_bits = 16
        cmd = bin(value)[2:].zfill(n_bits)[::-1]  # [::-1] for little endian.
        cmd = [int(char) for char in cmd]
        self.dio.output_word(target, cmd)


class antenna_device:
    """For the NANTEN2 telescope.

    Parameters
    ----------
    board_model
        Model number of Interface Digital I/O board.
    rsw_id
        Rotary Switch ID.

    """

    command_az_speed = command_el_speed = 0
    az_rate_d = el_rate_d = 0
    pre_hensa = [0, 0]
    ihensa = [0, 0]
    enc_before = [0, 0]
    pre_arcsec = [0, 0]
    t_now = t_past = 0
    p_coeff = [3.0, 3.0]
    i_coeff = [0.7, 0.7]
    d_coeff = [0.1, 0.1]
    dir_name = ""

    SPEED2RATE: float = (7 / 12) * (10000 / 3600)
    # Speed [arcsec/s] to servomotor rate.
    # 5250r of motor corresponds to 1r of antenna.
    # 1500rpm of motor corresponds to 1500/5250rpm = 2/7rpm = 12/7[deg/s] of antenna.
    # Command (we call it 'rate') for the servomotor is ratio of motor speed you desire
    # to the motor's max speed in permyriad.
    # e.g.) The rate corresponds to 100arcsec/s drive of antenna will be
    # (7/12)*100*(10000/3600).
    # Here (7/12)*(10000/3600) is the conversion factor `SPEED2RATE`.
    # *1: Unit 'r' is rotation.
    # *2: 5250 is the gear ratio for the NANTEN2 antenna drive.
    # *3: 1500rpm is max speed of the servomotor installed on the NANTEN2.
    LIMITS = [-270 * 3600, 270 * 3600]  # arcsec

    def __init__(
        self, board_model: int = 2724, rsw_id: int = 0, simulator: bool = False
    ) -> None:
        PIDController.ANGLE_UNIT = "arcsec"
        self._az = PIDController(
            pid_param=[self.p_coeff[0], self.i_coeff[0], self.d_coeff[0]],
            max_speed="1.5deg/s",
        )
        self._el = PIDController(
            pid_param=[self.p_coeff[1], self.i_coeff[1], self.d_coeff[1]],
            max_speed="1.5deg/s",
        )
        self.simulator = simulator
        if not self.simulator:
            self.driver = AntennaDriver(board_model, rsw_id)

    def _command(self, value: int, azel: Literal["az", "el"]) -> None:
        if not self.simulator:
            self.driver.command(value, azel)

    def init_speed(self) -> None:
        dummy = 0
        self._az.get_speed(dummy, dummy, stop=True)
        self._el.get_speed(dummy, dummy, stop=True)
        self._command(0, "az")
        self._command(0, "el")

    def set_pid_param(
        self, param: Dict[Literal["az", "el"], Tuple[float, float, float]]
    ) -> None:
        self._az.k_p, self._az.k_i, self._az.k_d = param["az"]
        self._el.k_p, self._el.k_i, self._el.k_d = param["el"]

    def move_azel(
        self,
        az_arcsec: float,
        el_arcsec: float,
        enc_az: float,
        enc_el: float,
        pid_param: Dict[str, Tuple[float, float, float]] = None,
        m_bStop: Literal["FALSE", "TRUE"] = "FALSE",
    ) -> Tuple[float, ...]:
        if pid_param is not None:
            self.set_pid_param(pid_param)

        if m_bStop == "FALSE":
            stop = False
        elif m_bStop == "TRUE":
            stop = True

        az_arcsec = optimum_angle(
            enc_az, az_arcsec, self.LIMITS, margin=40 * 3600, unit="arcsec"
        )

        speed_az = self._az.get_speed(az_arcsec, enc_az, stop=stop)
        speed_el = self._el.get_speed(el_arcsec, enc_el, stop=stop)
        self._command(int(speed_az * self.SPEED2RATE), "az")
        self._command(int(speed_el * self.SPEED2RATE), "el")

        self.t_now = self._az.time[Now]
        # May slightly different from `self._el.time[Now]`.

        self.az_rate_d = int(speed_az)  # Not rate, but speed in [arcsec/s].
        self.el_rate_d = int(speed_el)
        self.command_az_speed = self.az_rate_d
        self.command_el_speed = self.el_rate_d

        return (
            self._az.cmd_speed[Now] * 3600,
            self._az.k_p * self._az.error[Now] * 3600,
            self._az.k_i * self._az.error_integral * 3600 * self._az.dt,
            self._az.k_d
            * (self._az.error[Now] - self._az.error[Last])
            * 3600
            / self._az.dt,
            self._az.cmd_coord[Now] * 3600,
            self._az.enc_coord[Now] * 3600,
            self._az.error[Last] * 3600,
            self._az.error_integral * 3600,
            self._az.enc_coord[Last] * 3600,
            self._az.cmd_speed[Now] * 3600,
            self._el.k_p * self._az.error[Now] * 3600,
            self._el.k_i * self._az.error_integral * 3600 * self._az.dt,
            self._el.k_d
            * (self._az.error[Now] - self._az.error[Last])
            * 3600
            / self._az.dt,
            self._el.cmd_coord[Now] * 3600,
            self._el.enc_coord[Now] * 3600,
            self._el.error[Last] * 3600,
            self._el.error_integral * 3600,
            self._el.enc_coord[Last] * 3600,
            self._el.time[Now],
            self._el.time[Last],
        )  # All angle-related parameters are in arcsec.

    def emergency_stop(self) -> None:
        dummy = 0
        for _ in range(5):
            self._command(0, "az")
            self._command(0, "el")
            _ = self._az.get_speed(dummy, dummy, stop=True)
            _ = self._el.get_speed(dummy, dummy, stop=True)
            time.sleep(0.05)


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
    """PID calculation.

    ..deprecated:: v2.1.0
        This function will be removed in v4.0.0, because of implementation structure
        issue. Please use `AntennaDevice.calc_pid`.

    """
    calculator = PIDController(pid_param=[p_coeff, i_coeff, d_coeff])

    # Set `Last` parameters.
    calculator.time.push(t_past)
    calculator.cmd_coord.push(pre_arcsec / 3600)
    calculator.enc_coord.push(enc_before / 3600)
    calculator.error.push(pre_hensa / 3600)

    # Set new parameters.
    calculator.time.push(t_now)
    calculator.cmd_coord.push(target_arcsec / 3600)
    calculator.enc_coord.push(encoder_arcsec / 3600)
    calculator.error.push((target_arcsec - encoder_arcsec) / 3600)

    speed = calculator._calc_pid()

    error_diff = (calculator.error[Now] - calculator.error[Last]) / calculator.dt
    if abs(error_diff) * 3600 > 1:
        error_diff = 0

    return (
        speed,
        calculator.error_integral,
        calculator.k_p * calculator.error[Now],
        calculator.k_i * calculator.error_integral * calculator.dt,
        calculator.k_d * error_diff / calculator.dt,
    )
