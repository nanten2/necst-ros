#!/usr/bin/env python3

from shutil import register_unpack_format
import _observation


import time
import n_const
import argparse
from astropy import units as u


class OnTheFly(_observation.Observation):
    ObservationType = "LINEOTF"

    def ScanParameters(self):
        start_x = (
            self.obs["LambdaOff"] + self.obs["StartPositionX"]
        )  # Start position x[arcsec]
        start_y = (
            self.obs["deltaBeta"] + self.obs["StartPositionY"]
        )  # Start position y[arcsec]

        if self.obs["SCAN_DIRECTION"] == "X":
            dx, scan_space_y, dy, scan_space_x = self.Scan_direction()
        elif self.obs["SCAN_DIRECTION"] == "Y":
            dy, scan_space_x, dx, scan_space_y = self.Scan_direction()
        else:
            self.con.move_stop()
            raise ValueError("Error:direction")

        if self.obs["OTADEL"] == True:
            offset_dcos = 1
            dcos = 1
        else:
            offset_dcos = 0
            dcos = 0

        lamda = (n_const.constants.REST_FREQ[self.obs["MOLECULE_1"]]).to(
            u.m, equivalencies=u.spectral()
        )  # Observed line
        otflen = self.obs["scan_length"] / self.obs["integ_on"]
        scan_point = int(otflen)  # scan_point for 1 line
        ramp_time = self.obs["integ_on"] * self.obs["ramp_pixel"]  # time for ramping
        self.ramp_time = ramp_time
        total_count = int(self.obs["n"])  # total scan_line

        self.log.info("scan point is {}".format(scan_point))
        # if scan_point > int(scan_point):
        # print("!!ERROR scan number!!")

        self.start_x = start_x
        self.start_y = start_y
        self.dx = dx
        self.dy = dy
        self.scan_space_x = scan_space_x
        self.scan_space_y = scan_space_y
        self.lamda = lamda
        self.scan_point = scan_point
        self.ramp_time = ramp_time
        self.total_count = total_count
        self.dcos = dcos

    def Scan_direction(self):
        sample_spacing = self.obs["scan_velocity"] * self.obs["integ_on"]
        scan_spacing = self.obs["scan_spacing"]  # [arcsec]
        return sample_spacing, scan_spacing, 0 * u.arcsec, 0 * u.arcsec

    def run_onepoint(self, current_position: str, current_num, num, kwparams):
        kwparams_onepoint = {
            "x": self.obs["LambdaOff"].value,
            "y": self.obs["BetaOff"].value,
            "off_x": self.obs["deltaLambda"].value,
            "off_y": self.obs["deltaBeta"].value,
        }

        self.con.onepoint_move(**kwparams, **kwparams_onepoint)
        self.con.dome_track()

        self.log.info("check_track")
        self.con.antenna_tracking_check()
        self.con.dome_tracking_check()
        self.log.info("tracking OK")

        self.log.info(current_position)

        if current_position == "HOT":
            chopper_position = "in"
        else:
            chopper_position = "out"
        self.con.move_chopper(chopper_position)

        time.sleep(3)
        # "time.sleep(3)" is the substitute for this code;
        # status = con.read_status()
        # while status.Current_Hot != chopper_position.upper():
        #     log.info("wait hot_move")
        #     status = con.read_status()
        #     time.sleep(0.5)

        self.con.obs_status(
            active=True, current_num=current_num, current_position=current_position
        )
        self.log.info("get spectrum...")
        # temp = float(status.CabinTemp1)
        self.ger_spectrum(num=num, current_position=current_position)
        return time.time()

    def ger_spectrum(self, num, current_position):
        self.con.xffts_publish_flag(scan_num=num, obs_mode=current_position)
        time.sleep(self.obs["integ_off"].value)
        self.con.xffts_publish_flag(scan_num=num, obs_mode="")

    def timecheck(self, now, latest_calibtime, interval) -> bool:
        actual_interval = now - latest_calibtime
        return actual_interval > interval.value

    def RampStart(self, num):
        ramp_start_x = (
            (self.start_x + num * self.scan_space_x)
            - self.dx / self.obs["integ_on"] * self.ramp_time
            - self.dx / 2.0
        )  # ramp start position
        ramp_start_y = (
            (self.start_y + num * self.scan_space_y)
            - self.dy / self.obs["integ_on"] * self.ramp_time
            - self.dy / 2.0
        )  # ramp start position
        # dx/2 : for Nyquist sampling
        return ramp_start_x, ramp_start_y

    def ScanStart(self, ctime, delay, num):
        start_on = self.mjd(ctime + self.ramp_time + delay)
        end_scan = start_on + (self.obs["scan_length"] + self.ramp_time) / 24.0 / 3600.0
        off_x = self.start_x + num * self.scan_space_x
        off_y = self.start_y + num * self.scan_space_y
        return start_on, end_scan, off_x, off_y

    def mjd(self, time_sec):
        mjd_date = 40587 + time_sec / 24.0 / 3600.0
        return mjd_date

    def run_ramp(self, num, kwparams):
        ramp_start_x, ramp_start_y = self.RampStart(num=num)  # rampの始まり

        self.log.info("ramp_start tracking")
        kwparams_onepoint = {
            "x": self.obs["LambdaOn"].value,
            "y": self.obs["BetaOn"].value,
            "off_x": ramp_start_x.value,
            "off_y": ramp_start_y.value,
        }
        self.con.onepoint_move(**kwparams, **kwparams_onepoint)
        self.con.dome_track()  # temp

        self.con.antenna_tracking_check()
        self.con.dome_tracking_check()

        self.log.info("reach ramp_start")  # rampまで移動

    def run_otf(self, num, delay, kwparams):
        self.log.info(" OTF scan_start!! ")
        self.log.info("move ON")

        current_time = time.time()

        start_on, end_scan, off_x, off_y = self.ScanStart(
            ctime=current_time, delay=delay, num=num
        )
        # print(start_on)
        self.con.obs_status(
            active=True, current_num=self.scan_point * num, current_position="ON"
        )
        kwparams_scan = {
            "x": self.obs["LambdaOn"].value,
            "y": self.obs["BetaOn"].value,
            "dx": self.dx.value,
            "dy": self.dy.value,
            "dt": self.obs["integ_on"].value,
            "num": self.scan_point,
            "rampt": self.ramp_time.value,
            "delay": delay,
            "current_time": current_time,
            "off_x": off_x.value,
            "off_y": off_y.value,
            "hosei": "hosei_230.txt",
            "lamda": self.lamda.value,
            "limit": True,
        }
        self.con.otf_scan(**kwparams, **kwparams_scan)

        self.log.info("getting_data...")
        # d = con.oneshot_achilles(repeat = scan_point ,exposure = self.obs['integ_on'] ,stime = start_on)
        self.con.xffts_publish_flag(scan_num=num, obs_mode="ON")
        self.log.info("start_on : {}".format(start_on))

        while end_scan > 40587 + time.time() / (24.0 * 3600.0):
            # while obs['otflen']/24./3600. > 40587 + time.time()/(24.*3600.):
            time.sleep(0.001)
        self.con.xffts_publish_flag(obs_mode="", scan_num=num)

    def run(self, delay=3.0):
        self.ScanParameters()
        self.log.info("Start Observation")

        status = self.con.read_status()
        # savetime = status.Time

        latest_calibtime = 0

        # start observation
        # ---------------
        obsscript = __file__
        if self.obs["SCAN_DIRECTION"] == "X":
            params_obs = [
                True,
                self.ObservationType,
                obsscript,
                str(self.obsfile_path),
                self.obs["OBJECT"],
                self.scan_point,
                self.total_count,
                self.dx.value,
                self.scan_space_y.value,
                self.obs["integ_hot"].value,
                self.obs["integ_off"].value,
                self.obs["integ_on"].value,
                self.obs["SCAN_DIRECTION"],
            ]
        elif self.obs["SCAN_DIRECTION"] == "Y":
            params_obs = [
                True,
                self.ObservationType,
                obsscript,
                str(self.obsfile_path),
                self.obs["OBJECT"],
                self.scan_point,
                self.total_count.value,
                self.scan_space_x.value,
                self.dy.value,
                self.obs["integ_hot"].value,
                self.obs["integ_off"].value,
                self.obs["integ_on"].value,
                self.obs["SCAN_DIRECTION"],
            ]
        self.con.obs_status(*params_obs)

        for num in range(self.total_count):

            self.log.info("observation : {}".format(num))
            self.log.info("tracking start")
            self.con.move_stop()

            kwparams = {
                "coord": self.obs["COORD_SYS"],
                "offcoord": self.obs["COORD_SYS"],
                "dcos": self.dcos,
            }

            current_time = time.time()

            if self.timecheck(
                now=current_time,
                latest_calibtime=latest_calibtime,
                interval=self.obs["load_interval"],
            ):
                _ = self.run_onepoint(
                    current_position="HOT",
                    current_num=self.scan_point * num,
                    num=num,
                    kwparams=kwparams,
                )
                latest_calibtime = self.run_onepoint(
                    current_position="OFF",
                    current_num=self.scan_point * num,
                    num=num,
                    kwparams=kwparams,
                )

            self.con.move_stop()

            self.run_ramp(num=num, kwparams=kwparams)  # ramp

            self.run_otf(num=num, delay=delay, kwparams=kwparams)  # OTF scan

            # for _on in range(scan_point):
            #     self.log.info("{}".format(_on+1))

            self.log.info("stop")
            self.con.move_stop()

        # 最初と最後をhotではさむ
        self.run_onepoint(
            current_position="HOT",
            current_num=self.scan_point * num + 1,
            num=num,
            kwparams=kwparams,
        )

        # con.move_hot('out')
        self.con.move_chopper("out")
        time.sleep(3)

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
        self.con.move_stop()
        self.con.dome_stop()
        self.con.pub_loggerflag("")
        time.sleep(2)


if __name__ == "__main__":
    description = "On The Fly Observation"
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--obsfile", type=str, help="absolute path for obsfile")
    p.add_argument(
        "--delay",
        type=float,
        help="expected duration of coordinate calcuration",
        default=3.0,
    )
    args = p.parse_args()

    otf = OnTheFly(obsfile=args.obsfile)
    otf.run(delay=args.delay)
