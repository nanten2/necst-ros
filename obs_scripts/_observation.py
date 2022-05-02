#!/usr/bin/env python3

import abc
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from types import FrameType
from typing import ClassVar, Dict, List, NoReturn, Union

import astropy.units as u
from neclib import utils
from neclib.interfaces import console_logger
from neclib.parameters import ObsParams

sys.path.append("/home/amigos/ros/src/necst/scripts/controller")

import ROS_controller  # noqa: E402


HomeDir = Path.home()

UnitType = Union[str, u.Unit]


class Observation(abc.ABC):
    """Base class for observation scripts.

    Parameters
    ----------
    obsfile
        File name (not path) of observation spec file.
    verbose
        The higher the value is, the more messages appear on terminal. Valid values are
        in range [0, 50].

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
        UNIX time the observation was initialized (not the time ``run`` was called).

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

    def __init__(self, obsfile: str = None, verbose: int = 20) -> None:
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

        self.start_time = time.time()
        self.str_now = datetime.utcfromtimestamp(self.start_time)

        # Logger and database set-up.
        self.logger = self.init_logger(verbose)
        """Print messages to terminal. ``[debug|info|warning|error|critical|obslog]``"""
        self.fileconfig()

        self.logger.obslog(str(sys.argv))

        # Backwards compatible aliases.
        self.con = self.ctrl
        """``ROSController`` instance, which handles any instructions to any devices."""
        self.obs = self.params
        """``ObsParams`` instance, which contains the parameters of the observation."""
        self.log = self.logger
        """Print messages to terminal. ``[debug|info|warning|error|critical|obslog]``"""

    def init_logger(self, verbose: int) -> console_logger.ConsoleLogger:
        self.log_path = self.LogDir / f"{self.str_now.strftime('%Y%m%d')}.txt"
        obslog_path = self.log_path.parent / ("obs_" + {self.log_path.name})

        logger = console_logger.getLogger(
            __name__,
            file_path=self.log_path,
            obslog_file_path=obslog_path,
            min_level=int(utils.clip(50 - verbose, 0, 50)),
        )
        return logger

    def fileconfig(self) -> None:
        if self.params is not None:
            _spectra = self.params.get("MOLECULE_1", "")
            _target = self.params.get("OBJECT", "")
            db_name = f"n{self.str_now.strftime('%Y%m%d%H%M%S')}_{_spectra}_{_target}"
        else:
            db_name = f"n{self.str_now.strftime('%Y%m%d%H%M%S')}_{self.ObservationType}"

        db_path = self.DataDir / db_name
        self.logger.info(f"mkdir {db_path}")
        db_path.mkdir(parents=True, exist_ok=False)
        self.logger.obslog(f"savedir : {db_path}", 1)

        xffts_datapath = db_path / "xffts.ndf"
        self.ctrl.pub_loggerflag(str(db_path))

        self.logger.debug(f"obsfile : {self.obsfile_path}")
        self.logger.debug(f"log_path : {self.log_path}")
        self.logger.debug(f"dirname : {db_name}")
        self.logger.debug(f"xffts : {xffts_datapath}")

    def signal_handler(self, number: int, frame: FrameType) -> NoReturn:
        self.logger.warn("!! ctrl + C !!")
        self.logger.warn("STOP DRIVE")
        self.ctrl.move_stop()
        self.ctrl.dome_stop()
        self.ctrl.obs_status(active=False)
        _scan_num = getattr(self, "scan_num", -1)
        self.ctrl.xffts_publish_flag(obs_mode="", scan_num=_scan_num)
        self.ctrl.pub_loggerflag("")
        time.sleep(2)
        self.logger.obslog("STOP OBSERVATION", 1)
        time.sleep(1)
        sys.exit()

    @abc.abstractmethod
    def run(self) -> None:
        raise NotImplementedError

    @property
    def obsfile_params(self) -> ObsParams:
        return self.params

    @property
    def obsfile_path(self) -> Path:
        return self._obsfile_path


if __name__ == "__main__":
    raise RuntimeError(
        "This script cannot be executed as this is base class definition."
    )
