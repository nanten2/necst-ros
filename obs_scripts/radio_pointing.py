#!/usr/bin/env python3

import math
import time
from typing import List

import numpy as np
from neclib.typing import Literal
from neclib.utils import counter

from _observation import Observation


class RadioPointing(Observation):

    ObservationType = "LINECROSS"
    ParameterUnits = {
        "deg": [
            "LamdaOn",
            "BetaOn",
            "LamdaOff",
            "BetaOff",
            "GridAz",
            "GridEl",
            "OffsetAz",
            "OffsetEl",
        ],
        "s": ["integ_on", "integ_off", "integ_hot", "load_interval"],
    }

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self.need_x_to_az_conversion = int(self.params.OTADEL)

    def grid_offset(self, spacing: float, n_per_axis: int) -> List[float]:
        """Calculate offsets."""
        signed_idx = np.arange(n_per_axis) - ((n_per_axis - 1) / 2)
        return signed_idx * spacing

    def wait_for_chopper_move(self):
        time.sleep(3)

    def check_interval(self, last_calib_time: float, interval: float) -> bool:
        """Check the interval has passed or not."""
        time_passed = time.time() - last_calib_time
        return time_passed > interval

    def get_spectra(
        self,
        exposure: float,
        obsmode: str,
        pt_num: int,
        x_offset: float,
        y_offset: float,
    ) -> None:
        """Get spectra."""
        self.ctrl.xffts_publish_flag(
            scan_num=pt_num, obs_mode=obsmode, lamdel=x_offset, betdel=y_offset
        )
        time.sleep(exposure)
        self.ctrl.xffts_publish_flag()

    def run_calibration(
        self, mode: Literal["hot", "off"], num: int, x_offset: float, y_offset: float
    ) -> None:
        """Run calibration observation."""
        chopper_position = {"hot": "in", "off": "out"}

        self.log.info(f"Start {mode.upper()} observation...")
        self.ctrl.move_chopper(chopper_position[mode.lower()])
        self.wait_for_chopper_move()

        self.ctrl.onepoint_move(
            *(self.params.LamdaOff, self.params.BetaOff, self.params.COORD_SYS),
            off_x=self.params.OffsetAz,
            off_y=self.params.OffsetEl,
            dcos=self.need_x_to_az_conversion,
        )
        self.ctrl.antenna_tracking_check()
        self.ctrl.dome_tracking_check()
        self.log.info("Tracking OK")

        self.ctrl.obs_status(
            active=True,
            current_num=num,
            current_position=mode.upper(),
        )
        status = self.ctrl.read_status()
        self.log.info(f"Temperature: {status.CabinTemp1}")

        self.get_spectra(self.params.integ_hot, mode, num, x_offset, y_offset)

    def run(self, n_observations: int = None) -> None:
        """Run the observation.

        Parameters
        ----------
        n_observations
            Number of observations to run. If None, run until ctrl-C is pressed.

        """
        calib_time = 0  # Initial value to ensure calibration observation to run first.
        n_pts_per_axis = math.ceil(self.params.METHOD / 2)

        self.log.info(f"Start {self.ObservationType} observation...")
        self.ctrl.move_stop()
        self.ctrl.xffts_publish_flag()

        self.ctrl.dome_track()

        self.ctrl.obs_status(
            active=True,
            obsmode=self.ObservationType,
            obs_script=__file__,
            target=self.params.OBJECT,
            num_on=n_pts_per_axis,
            num_seq=1,
            xgrid=self.params.GridAz,
            ygrid=self.params.GridEl,
            exposure_hot=self.params.integ_hot,
            exposure_off=self.params.integ_off,
            exposure_on=self.params.integ_on,
        )

        for n in counter(n_observations):
            offset_x_scan = [
                (offset, 0)
                for offset in self.grid_offset(self.params.GridAz, n_pts_per_axis)
            ]
            offset_y_scan = [
                (0, offset)
                for offset in self.grid_offset(self.params.GridEl, n_pts_per_axis)
            ]
            for i, (x_offset, y_offset) in enumerate(offset_x_scan + offset_y_scan):
                pt_idx = n * self.params.METHOD + i + 1

                if self.check_interval(calib_time, self.params.load_interval):
                    calib_time = self.run_calibration("hot", pt_idx, x_offset, y_offset)
                    _ = self.run_calibration("off", pt_idx, x_offset, y_offset)

                self.ctrl.onepoint_move(
                    *(self.params.LamdaOn, self.params.BetaOn, self.params.COORD_SYS),
                    off_x=self.params.OffsetAz + x_offset,
                    off_y=self.params.OffsetEl + y_offset,
                    offcoord="horizontal",
                    dcos=self.need_x_to_az_conversion,
                )
                self.ctrl.antenna_tracking_check()
                self.ctrl.dome_tracking_check()
                self.log.info("Tracking OK")


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(description="Radio Pointing, line observation")
    p.add_argument(
        "--obsfile",
        type=str,
        help="Name of observation file, 'line_*pt_*.obs.toml'",
        required=True,
    )
    p.add_argument(
        "--n_obs",
        "-n",
        type=int,
        help="Number of the observation repeated. Default: infinite",
        required=False,
        default=None,
    )
    args = p.parse_args()

    observer = RadioPointing(args.obsfile)
    observer.run(args.n_obs)
