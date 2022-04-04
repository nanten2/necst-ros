#!/usr/bin/env python3

import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import ClassVar

from n_const import ObsParams
from neclib.typing import PathLike

sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")

import logger  # noqa: E402
import ROS_controller  # noqa: E402

HomeDir = Path.home()


class Observation:
    """Base class for observation scripts.

    Notes
    -----
    Databases are saved under ``DataBaseDir / ObservationType``.

    """

    ObservationType: ClassVar[str]
    """Kind of this observation."""

    ObsfileDir: PathLike = HomeDir / "necst-obsfiles"
    """Directory which contain observation spec files."""
    LogDir: PathLike = HomeDir / "log"
    """Directory into which observation command log are saved."""
    DataBaseDir: PathLike = HomeDir / "data" / "observation"
    """Parent directory into which observation database is saved."""

    def __init__(self, obsfile: PathLike = None) -> None:
        self.DataDir = self.DataBaseDir / self.ObservationType

        self.con = ROS_controller.controller()
        if obsfile is not None:
            self._obsfile_path = self.ObsfileDir / obsfile
            self.obs = ObsParams.from_file(self._obsfile_path)
        else:
            self.obs = None

        signal.signal(signal.SIGINT, self.signal_handler)
        self.con.get_authority()
        self.now = datetime.utcnow()
        self.init_logger()
        self.fileconfig()

    def init_logger(self) -> None:
        self.log_path = self.LogDir / f"{self.now.strftime('%Y%m%d')}.txt"

        self.logger = logger.logger(__name__, filename=self.log_path)
        self.log = self.logger.setup_logger()
        self.logger.obslog(sys.argv)

        self.start_time = time.time()

    def fileconfig(self) -> None:
        if self.obs is not None:
            _spectra = self.obs.get("MOLECULE_1", "")
            _target = self.obs.get("OBJECT", "")
            db_name = f"n{self.now.strftime('%Y%m%d%H%M%S')}_{_spectra}_{_target}"
        else:
            db_name = f"n{self.now.strftime('%Y%m%d%H%M%S')}_{self.ObservationType}"

        db_path = self.DataDir / db_name
        self.log.info(f"mkdir {db_path}")
        os.makedirs(db_path)
        self.logger.obslog(f"savedir : {db_path}", lv=1)

        xffts_datapath = db_path / "xffts.ndf"
        self.con.pub_loggerflag(str(db_path))

        self.log.debug(f"obsdir : {self.obsfile_path}")
        self.log.debug(f"log_path : {self.log_path}")
        self.log.debug(f"dirname : {db_name}")
        self.log.debug(f"xffts : {xffts_datapath}")

    def signal_handler(self):
        self.log.warn("!! ctrl + C !!")
        self.log.warn("STOP DRIVE")
        self.con.move_stop()
        self.con.dome_stop()
        self.con.obs_status(active=False)
        _scan_num = getattr(self, "scan_num", -1)
        self.con.xffts_publish_flag(obs_mode="", scan_num=_scan_num)
        self.con.pub_loggerflag("")
        time.sleep(2)
        self.logger.obslog("STOP OBSERVATION", lv=1)
        time.sleep(1)
        sys.exit()

    def run(self):
        raise NotImplementedError

    @property
    def obsfile_params(self):
        return self.params

    @property
    def obsfile_path(self):
        return getattr(self, "_obsfile_path", "")


if __name__ == "__main__":
    raise RuntimeError(
        "This script cannot be executed as this is base class definition."
    )
