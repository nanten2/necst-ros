#!/usr/bin/env python3

import time
from _observation import Observation


class Rsky(Observation):
    def __init__(self):

        ObservationType = "Rsky"
        super().__init__(ObservationType=ObservationType)

    def run(self, integ_time):
        print("Start R SKY observation")

        self.con.move_chopper("in")
        time.sleep(3)  # Temporarily

        status = self.con.read_status()
        hot_status = status.Current_Hot

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

        self.logger.obslog(
            "Observation End : observation time : {:.2f} [min]".format(
                (time.time() - self.start_time) / 60
            ),
            lv=1,
        )
        self.log.info(
            "Observation End : observation time : {:.2f} [min]".format(
                (time.time() - self.start_time) / 60
            )
        )

        # con.move_hot('in')
        self.con.pub_loggerflag("")
        # self.con.pub_analyexec(savedir2, "rsky")


if __name__ == "__main__":
    rsky = Rsky()
    rsky.init_logger()
    rsky.fileconfig()
    rsky.signal_handler()
    rsky.run(integ_time=2)