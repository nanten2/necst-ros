#!/usr/bin/env python3

import math
import time
from typing import Any, Dict, Hashable, List, Tuple

import astropy.units as u
import neclib
import numpy as np
from neclib.typing import Literal
from neclib.utils import counter

from _observation import Observation

PossibleAngleUnits = [unit.to_string() for unit in u.deg.find_equivalent_units] + [
    unit for unit in u.deg.find_equivalent_units
]


class RadioPointing(Observation):
    """Radio line pointing observation.

    Parameters
    ----------
    coordsys_kwargs
        Keyword arguments for coordinate system conversion, such as ``obstime``,
        ``location``, etc.

    """

    ObservationType = "LINECROSS"
    ParameterUnits = {
        "deg": [
            "LamdaOn",
            "BetaOn",
            "LamdaOff",
            "BetaOff",
            "OffsetAz",
            "OffsetEl",
            "GridAz",
            "GridEl",
        ],
        "s": ["integ_on", "integ_off", "integ_hot", "load_interval"],
    }

    def __init__(
        self, *args, coordsys_kwargs: Dict[Hashable, Any] = {}, **kwargs
    ) -> None:
        super().__init__(*args, **kwargs)

        self.num_points_per_axis = math.ceil(self.params.METHOD / 2)
        self._used_angular_units = [
            unit for unit in self.ParameterUnits.values() if unit in PossibleAngleUnits
        ] + [unit for unit in self.ParameterUnits.keys() if unit in PossibleAngleUnits]
        angular_unit = self._used_angular_units[0]

        # Pre-calculate parameters
        self.interval = {
            "hot": self._get_interval("hot"),
            "off": self._get_interval("off"),
        }
        self.off_point_coord = self.params.off_point_coord(
            unit=angular_unit, **coordsys_kwargs
        )
        self.on_point_coord = (
            self.params.val.LamdaOn,
            self.params.val.BetaOn,
            self.params.COORD_SYS,
        )
        self.lonlat_applied = True  # Already converted above.

        # Initial time 0 is to ensure the calibration observations to run first.
        self.last_calib_time = {"off": 0, "hot": 0}
        self.point_count = 0
        self.last_calib_point = {"off": 0, "hot": 0}

    def _get_interval(self, mode: Literal["hot", "off"]) -> Tuple[float, str]:
        getter = getattr(self.params, f"{mode}_observation_interval")
        try:
            return getter(unit="s")
        except u.UnitConversionError:
            return getter(unit="point", points_per_scan=self.num_points_per_axis)

    def _arcsec(self, value):
        """Scale parameter value to arcsec.

        The only reason this function was implemented is ROS_controller assumes
        coordinate is to be given in "deg", but coordinate offsets in "arcsec", though
        both the parameters are angular quantity.
        I hope the inconsistent and bug prone parameter handling to be fixed and this
        method to be removed ASAP.

        """
        if sum(self._used_angular_units) > 1:
            raise ValueError("Using multiple angular units isn't supported.")
        if isinstance(value, u.Quantity) and (not value.unit.is_unity()):
            raise ValueError("Quantity with physical dimension isn't supported.")
        return value * neclib.utils.angle_conversion_factor(
            self._used_angular_units[0], "arcsec"
        )

    def grid_offset(self, spacing: float, n_per_axis: int) -> List[float]:
        """Calculate offsets."""
        signed_idx = np.arange(n_per_axis) - ((n_per_axis - 1) / 2)
        return signed_idx * spacing

    def wait_for_chopper_move(self):
        time.sleep(3)

    def check_interval(self, mode: Literal["hot", "off"]) -> bool:
        """Check the interval has passed or not."""
        _interval, _type = self.interval[mode]
        if _type == "time":
            time_passed = time.time() - self.last_calib_time[mode]
            return time_passed > _interval
        elif _type == "point":
            points_observed = self.point_count - self.last_calib_point[mode]
            return points_observed > _interval
        raise ValueError("Interval type cannot be interpreted.")

    @property
    def scan_count(self) -> int:
        """Zero-based count of axes currently observing."""
        return self.point_count // self.num_points_per_axis

    def get_spectra(
        self,
        exposure: float,
        obsmode: str,
        pt_idx: int,
        x_offset: float,
        y_offset: float,
    ) -> None:
        """Get spectra."""
        self.ctrl.xffts_publish_flag(
            scan_num=pt_idx,
            obs_mode=obsmode,
            lamdel=self._arcsec(x_offset),
            betdel=self._arcsec(y_offset),
        )
        time.sleep(exposure)
        self.ctrl.xffts_publish_flag()

    def drive_and_wait(
        self,
        to: Tuple[float, float, str],
        offset: Tuple[float, float, str],
        lonlat_applied: bool,
    ) -> None:
        offset_kw = {"off_x": offset[0], "off_y": offset[1], "offcoord": offset[2]}
        self.ctrl.onepoint_move(*to, **offset_kw, dcos=int(not lonlat_applied))
        self.ctrl.antenna_tracking_check()
        self.ctrl.dome_tracking_check()
        self.log.info("Tracking OK")

    def run_calibration(
        self, mode: Literal["hot", "off"], pt_idx: int, x_offset: float, y_offset: float
    ) -> None:
        """Run calibration observation."""
        chopper_position = {"hot": "in", "off": "out"}

        self.log.info(f"Start {mode.upper()} observation...")
        self.ctrl.move_chopper(chopper_position[mode.lower()])
        self.wait_for_chopper_move()

        if mode.lower() == "off":
            self.drive_and_wait(
                self.off_point_coord,
                (self.params.val.OffsetAz, self.params.val.OffsetEl, "horizontal"),
                self.lonlat_applied,
            )
        self.ctrl.obs_status(
            active=True,
            current_num=pt_idx,
            current_position=mode.upper(),
        )
        status = self.ctrl.read_status()
        self.log.info(f"Temperature: {status.CabinTemp1}")

        self.get_spectra(self.params.integ_hot, mode, pt_idx, x_offset, y_offset)

        self.last_calib_point[mode] = self.point_count
        self.last_calib_time[mode] = time.time()

    def run(self) -> None:
        """Run the observation.

        Parameters
        ----------
        n_observations
            Number of observations to run. If None, run until ctrl-C is pressed.

        """
        self.log.info(f"Start {self.ObservationType} observation...")
        self.ctrl.move_stop()
        self.ctrl.xffts_publish_flag()

        self.ctrl.dome_track()

        self.ctrl.obs_status(
            active=True,
            obsmode=self.ObservationType,
            obs_script=__file__,
            target=self.params.val.OBJECT,
            num_on=self.num_points_per_axis,
            num_seq=1,
            xgrid=self.params.val.GridAz,
            ygrid=self.params.val.GridEl,
            exposure_hot=self.params.val.integ_hot,
            exposure_off=self.params.val.integ_off,
            exposure_on=self.params.val.integ_on,
        )

        for _ in counter(self.params.N):
            args_az = (self.params.val.GridAz, self.num_points_per_axis)
            obs_point_offset_az = list(
                map(lambda offset: (offset, 0), self.grid_offset(*args_az))
            )
            args_el = (self.params.val.GridEl, self.num_points_per_axis)
            obs_point_offset_el = list(
                map(lambda offset: (0, offset), self.grid_offset(*args_el))
            )

            for offset_az, offset_el in obs_point_offset_az + obs_point_offset_el:
                self.point_count += 1

                if self.check_interval("hot"):
                    self.run_calibration("hot", self.point_count, offset_az, offset_el)
                if self.check_interval("off"):
                    self.run_calibration("off", self.point_count, offset_az, offset_el)

                self.drive_and_wait(
                    self.on_point_coord,
                    (
                        self.params.val.OffsetAz + offset_az,
                        self.params.val.OffsetEl + offset_el,
                        "horizontal",
                    ),
                    self.lonlat_applied,
                )
                self.get_spectra(
                    self.params.val.integ_on,
                    "on",
                    self.point_count,
                    offset_az,
                    offset_el,
                )


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(description="Radio Pointing, line observation")
    p.add_argument(
        "--obsfile",
        type=str,
        help="Name of observation file, 'line_*pt_*.obs.toml'",
        required=True,
    )
    args = p.parse_args()

    observer = RadioPointing(args.obsfile)
