#!/usr/bin/env python3

import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import ClassVar

from n_const import ObsParam
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

    ObservationType: ClassVar[str] = ""
    """Kind of this observation."""

    ObsfileDir: PathLike = HomeDir / "necst-obsfiles"
    """Directory which contain observation spec files."""
    LogDir: PathLike = HomeDir / "log"
    """Directory into which observation command log are saved."""
    DataBaseDir: PathLike = HomeDir / "data" / "observation"
    """Parent directory into which observation database is saved."""

    def __init__(self, obsfile: PathLike) -> None:
        self.DataDir = self.DataBaseDir / self.ObservationType

        self.con = ROS_controller.controller()
        self.obsfile_path = self.ObsfileDir / obsfile
        self.obs = ObsParam.from_file(self.obsfile_path)

        signal.signal(signal.SIGINT, self.handler)

        self.now = datetime.utcnow()
        self.init_logger()
        self.fileconfig()

    def init_logger(self) -> None:
        self.log_path = self.LogDir / f"{self.now.strftime('%Y%m%d')}.txt"

        self.logger = logger.logger(__name__, filename=self.log_path)
        self.log = logger.setup_logger()
        logger.obslog(sys.argv)

        self.start_time = time.time()

    def fileconfig(self) -> None:
        _spectra = self.obs.get("MOLECULE_1", "")
        _target = self.obs.get("OBJECT", "")
        db_name = "n" + self.now.strftime("%Y%m%d%H%M%S") + _spectra + _target

        db_path = self.DataDir / db_name
        self.log.info(f"mkdir {db_path}")
        os.makedirs(db_path)
        logger.obslog(f"savedir : {db_path}", lv=1)

        xffts_datapath = db_path / "xffts.ndf"

        self.log.debug(f"obsdir : {self.obsfile_path}")
        self.log.debug(f"log_path : {self.log_path}")
        self.log.debug(f"dirname : {db_name}")
        self.log.debug(f"xffts : {xffts_datapath}")

    def Handler(self):
        self.log.warn("!! ctrl + C !!")
        self.log.warn("STOP DRIVE")
        self.con.move_stop()
        self.con.dome_stop()
        self.obs_status(active=False)
        self.xffts_publish_flag(obs_mode="", scan_nun=self.scan_num)
        self.pub_loggerflag("")
        time.sleep(2)
        logger.obslog("STOP OBSERVATION", lv=1)
        time.sleep(1.0)
        sys.exit()

    def run(self):
        raise NotImplementedError

    @property
    def obsfileparams(self):
        return self.obs

    @property
    def inspect_obsfile_path(self):
        return self.obsfile_path


if __name__ == "__main__":
    pass
