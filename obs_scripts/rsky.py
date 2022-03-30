#!/usr/bin/env python3

import time
from _observation import Observation

import argparse


class RSky(Observation):
    """
    An observing module for R-SKY observation, which provides Tsys
    measurement toward (az, el)=(45, 70))
    """

    ObservationType = "R-SKY"

    def __init__(self):
        # __init__ does not take any arguments(cf., obsfile) beacause in the R-SKY
        # observation module, all observation params are included in this module.

        super().__init__()

    def run(self, integ_time):

        print("Start R SKY observation")
        self.con.move_chopper("in")
        time.sleep(3)  # Temporarily
        status = self.con.read_status()
        hot_status = status.Current_Hot
        current_Az = status.Current_Az

        self.con.onepoint_move(x=current_Az, y=70)
        self.con.dome_tracking_check()
        self.log.info("dome track OK")
        self.concon.antenna_tracking_check()
        self.log.info("antenna track OK")

        print("HOT")
        print("hot_status ### ", hot_status)
        print("get spectrum...")

        self.con.pub_loggerflag(self.DataDir)
        self.con.xffts_publish_flag(obs_mode="HOT")
        time.sleep(integ_time)
        self.con.xffts_publish_flag()

        self.con.move_chopper("out")
        time.sleep(3)  # Temporarily

        status = self.con.read_status()
        hot_status = status.Current_Hot

        print("SKY")
        print("hot_status ### ", hot_status)
        print("get spectrum...")

        self.con.xffts_publish_flag(obs_mode="SKY")
        time.sleep(integ_time)
        self.con.xffts_publish_flag()

        log_contents = (
            "Observation End : observation time : "
            f"{(time.time() - self.start_time) / 60:.2f} [min]"
        )

        self.logger.obslog(log_contents, lv=1)
        self.log.info(log_contents)

        self.con.pub_loggerflag("")
        # self.con.pub_analyexec(savedir2, "rsky")


if __name__ == "__main__":
    description = "R sky Observation"
    p = argparse.ArgumentParser(description=description)
    p.add_argument(
        "--integ_time",
        type=float,
        help="Integration time for the R-sky obs.",
        default=2,
    )
    args = p.parse_args()

    rsky = RSky()
    rsky.run(integ_time=args.integ_time)
