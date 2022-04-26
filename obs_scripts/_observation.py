#!/usr/bin/env python3

import abc
import logging
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from types import FrameType
from typing import ClassVar, Dict, List, Union

import astropy.units as u
from neclib.parameters import ObsParams

sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")

import logger  # noqa: E402
import ROS_controller  # noqa: E402


HomeDir = Path.home()

UnitType = Union[str, u.Unit]


class Observation(abc.ABC):
    """Base class for observation scripts.

    Parameters
    ----------
    obsfile
        File name (not path) of observation spec file.

    Notes
    -----
    Databases are saved under ``DatabaseDir / ObservationType``.

    Attributes
    ----------
    ctrl, con
        ``ROSController`` instance, to which any instructions to devices are passed.
    params, obs
        ``ObsParams`` instance, which contains the parameters of the observation.
    log
        ``logging.Logger`` instance, which prints the log messages to the terminal via
        ``[debug|info|warning|error|critical]`` methods.
    logger
        ``logger`` instance, which saves the log messages to the log file via ``obslog``
        method.
    start_time
        UNIX time the observation was initialized (not the time ``run`` is called).

    """

    ObservationType: ClassVar[str]
    """Kind of this observation."""
    ParameterUnits: ClassVar[Union[Dict[str, UnitType], Dict[UnitType, List[str]]]] = {}

    ObsfileDir: ClassVar[Path] = HomeDir / "necst-obsfiles"
    """Directory which contain observation spec files."""
    LogDir: ClassVar[Path] = HomeDir / "log"
    """Directory into which observation command log are saved."""
    DatabaseDir: ClassVar[Path] = HomeDir / "data" / "observation"
    """Parent directory into which observation database is saved."""

    def __init__(self, obsfile: str = None) -> None:
        self.DataDir = self.DatabaseDir / self.ObservationType

        self.ctrl = ROS_controller.controller()
        """``ROSController`` instance, which handles any instructions to any devices."""

        self._obsfile_path = None if obsfile is None else self.ObsfileDir / obsfile
        ObsParams.ParameterUnit = self.ParameterUnits
        self.params = (
            ObsParams() if obsfile is None else ObsParams.from_file(self._obsfile_path)
        )
        """``ObsParams`` instance, which contains the parameters of the observation."""

        signal.signal(signal.SIGINT, self.signal_handler)
        self.ctrl.get_authority()
        self.now = datetime.utcnow()
        self.init_logger()
        self.fileconfig()

        # Backwards compatible aliases.
        self.con = self.ctrl
        """``ROSController`` instance, which handles any instructions to any devices."""
        self.obs = self.params
        """``ObsParams`` instance, which contains the parameters of the observation."""

        # Variable annotation
        self.logger = self.logger
        """Save log messages to the log file via ``obslog`` method."""
        self.log: logging.Logger = self.log
        """Print message to terminal. Method: ``[debug|info|warning|error|critical]``"""

    def init_logger(self) -> None:
        self.log_path = self.LogDir / f"{self.now.strftime('%Y%m%d')}.txt"

        self.logger = logger.logger(__name__, filename=self.log_path)
        self.log = self.logger.setup_logger()

        self.logger.obslog(sys.argv)
        self.start_time = time.time()

    def fileconfig(self) -> None:
        if self.params is not None:
            _spectra = self.params.get("MOLECULE_1", "")
            _target = self.params.get("OBJECT", "")
            db_name = f"n{self.now.strftime('%Y%m%d%H%M%S')}_{_spectra}_{_target}"
        else:
            db_name = f"n{self.now.strftime('%Y%m%d%H%M%S')}_{self.ObservationType}"

        db_path = self.DataDir / db_name
        self.log.info(f"mkdir {db_path}")
        os.makedirs(db_path)
        self.logger.obslog(f"savedir : {db_path}", lv=1)

        xffts_datapath = db_path / "xffts.ndf"
        self.ctrl.pub_loggerflag(str(db_path))

        self.log.debug("obsdir :", self.obsfile_path)
        self.log.debug("log_path :", self.log_path)
        self.log.debug("dirname :", db_name)
        self.log.debug("xffts :", xffts_datapath)

    def signal_handler(self, number: int, frame: FrameType):
        self.log.warn("!! ctrl + C !!")
        self.log.warn("STOP DRIVE")
        self.ctrl.move_stop()
        self.ctrl.dome_stop()
        self.ctrl.obs_status(active=False)
        _scan_num = getattr(self, "scan_num", -1)
        self.ctrl.xffts_publish_flag(obs_mode="", scan_num=_scan_num)
        self.ctrl.pub_loggerflag("")
        time.sleep(2)
        self.logger.obslog("STOP OBSERVATION", lv=1)
        time.sleep(1)
        sys.exit()

    @abc.abstractmethod
    def run(self):
        raise NotImplementedError

    @property
    def obsfile_params(self):
        return self.params

    @property
    def obsfile_path(self):
        return self._obsfile_path


if __name__ == "__main__":
    raise RuntimeError(
        "This script cannot be executed as this is base class definition."
    )
