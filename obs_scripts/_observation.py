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
    ctrl, con: ROS_controller.controller
        ``controller`` instance, to which any instructions to devices are passed.
    params, obs: neclib.parameters.ObsParams
        ``ObsParams`` instance, which contains the parameters of the observation.
    logger, log: neclib.interfaces.ConsoleLogger
        ``ConsoleLogger`` instance, which prints the log messages on the terminal via
        ``[debug|info|warning|error|critical|obslog]`` methods.
    start_time: float
        UNIX time the observation was initialized (not the time ``run`` was called).
    log_path: pathlib.Path
        Path to a file which contains log messages.
    obslog_path: pathlib.Path
        Path to a file which contains observation summary.
    db_path: pathlib.Path
        Path to a directory which contains observation and drive data.

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
        self.start_time = time.time()
        self.str_now = datetime.utcfromtimestamp(self.start_time)

        # Get observation parameters.
        self._obsfile_path = None if obsfile is None else self.ObsfileDir / obsfile
        ObsParams.ParameterUnit = self.ParameterUnits
        self.params = (
            ObsParams() if obsfile is None else ObsParams.from_file(self._obsfile_path)
        )
        """``ObsParams`` instance, which contains the parameters of the observation."""

        # Set up controller.
        self.ctrl = ROS_controller.controller()
        """``ROSController`` instance, which handles any instructions to any devices."""
        self.ctrl.get_authority()
        signal.signal(signal.SIGINT, self.signal_handler)

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
        self.obslog_path = (
            self.log_path.parent / f"{self.log_path.stem}_summary{self.log_path.suffix}"
        )

        logger = console_logger.getLogger(
            __name__,
            file_path=self.log_path,
            obslog_file_path=self.obslog_path,
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

        self.db_path = self.DatabaseDir / self.ObservationType / db_name
        self.logger.debug(f"mkdir {self.db_path}")
        self.db_path.mkdir(parents=True, exist_ok=False)

        self.ctrl.pub_loggerflag(str(self.db_path))

        self.logger.info(f"obsfile : {self.obsfile_path}")
        self.logger.debug(f"log_path : {self.log_path}")
        self.logger.info(f"database : {db_name}")

        self.logger.obslog(f"savedir : {self.db_path}", 1)

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
    def obsfile_path(self) -> Path:
        return self._obsfile_path


if __name__ == "__main__":
    raise RuntimeError(
        "This script cannot be executed as this is base class definition."
    )
