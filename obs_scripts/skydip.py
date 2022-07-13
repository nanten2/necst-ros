#!/usr/bin/env python3

import time
from typing import List

from _observation import Observation


class Skydip(Observation):
    """Skydip observation.

    Parameters
    ----------
    integ
        HOT観測とSKY観測の積分時間

    """

    ObservationType = "SKYDIP"
    Elevation: List[float] = [80, 70, 60, 45, 30, 25, 20]

    def __init__(self, *args, integ: float, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.integ = integ

    def wait_for_chopper_move(self) -> None:
        time.sleep(3)

    def get_spectra(
        self,
        exposure: float,
        obsmode: str,
        pt_idx: int,
    ) -> None:
        self.ctrl.xffts_publish_flag(
            scan_num=pt_idx,
            obs_mode=obsmode,
        )
        time.sleep(exposure)
        self.ctrl.xffts_publish_flag()

    def run_hot_observation(self) -> None:
        self.log.info("Start HOT observation...")
        self.ctrl.move_chopper("in")
        self.wait_for_chopper_move()

        self.ctrl.obs_status(active=True, current_position="HOT")

        self.get_spectra(self.integ, "HOT", 99)

        self.ctrl.move_chopper("out")

    def run(self) -> None:
        """Run the observation."""
        self.log.info(f"Start {self.ObservationType} observation...")
        self.ctrl.move_stop()
        self.ctrl.xffts_publish_flag()

        self.ctrl.dome_track()

        self.ctrl.obs_status(
            active=True,
            obsmode=self.ObservationType,
            obs_script=__file__,
            exposure_hot=self.integ,
            exposure_on=self.integ,
        )

        self.run_hot_observation()

        self.log.info("Start SKY observation...")
        status = self.con.read_status()
        current_Az = status.Current_Az

        for elevation in self.Elevation:
            self.ctrl.onepoint_move(current_Az, elevation)
            self.ctrl.dome_tracking_check()
            self.log.debug("dome track OK")
            self.ctrl.antenna_tracking_check()
            self.log.debug("antenna track OK")
            self.ctrl.move_stop()
            self.get_spectra(self.integ, "SKY", elevation)

        self.ctrl.move_stop()
        self.log.info(f"{self.ObservationType.upper()} observation finished.")


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(description="Skydip observation")
    p.add_argument(
        "--integ", type=float, default=1.0, help="Integration time (sec). Default = 1.0"
    )
    p.add_argument(
        "-v",
        "--verbose",
        action="count",
        default=0,
        help=(
            "Verbosity level of log messages appear on terminal."
            "To show all messages, use '-vvv'."
        ),
    )
    args = p.parse_args()

    observer = Skydip(integ=args.integ, verbose=20 + args.verbose * 10)
    observer.run()
    observer.ctrl.release_authority()
