#!/usr/bin/env python3

import time
import sys
import logger
import os
import signal
from pathlib import Path

from n_const.obsparams import ObsParam
from datetime import datetime, timedelta

sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")

import ROS_controller


class ObsBase(object):
    def __init__(
        self,
        obsfile: str,
    ) -> None:

        ObsFileDir: os.PathLike = Path("/home/amigos/necst-obsfiles")


        self.con = ROS_controller.controller()
        self.obsfilepath = ObsFileDir / obsfile
        self.obs = ObsParam.from_file(self.obsfilepath)

        signal.signal(signal.SIGINT, self.handler)

        self.now = datetime.utcnow()
        self.init_logger()
        self.fileconfig()

    def init_logger(self) -> None:
        self.log_path = Path(os.environ["HOME"]) + self.now.strftime("%Y%m%d") + ".txt"
        self.logger = logger.logger(__name__, filename=self.log_path)
        self.log = logger.setup_logger()
        logger.obslog(sys.argv)
        self.start_time = time.time()

    def fileconfig(self, name: str) -> None:
        datahome = './observation/otf'
        dirname = (
            "n"
            + name
            + self.now.strftime("%Y%m%d%H%M%S")
            + str(self.obs["molecule_1"])
            + str(self.obs["transiti_1"].split("=")[1])
            + str(self.obs["object"])
        )
        savedir = os.path.join(datahome, name, dirname)
        self.log.info("mkdir {savedir}".format(**locals()))
        os.makedirs(savedir)
        logger.obslog("savedir : {}".format(savedir), lv=1)

        xffts_datapath = os.path.join(savedir, "xffts.ndf")

        self.log.debug("obsdir : {}".format(self.obsfilepath))
        self.log.debug("log_path : {}".format(self.logsavepath))
        self.log.debug("dirname : {}".format(dirname))
        self.log.debug("xffts : {}".format(xffts_datapath))

    def Handler(self):
        self.log.warn("!!ctrl+C!!")
        self.log.warn("STOP MOVING")
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
    def inspect_obsfilepath(self):
        return self.obsfilepath