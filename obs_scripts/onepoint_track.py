#! /usr/bin/env python3

import os
import time
import math
import argparse
import signal

# from pyslalib import slalib
import ccd_old
import coord
import ROS_controller
import sys
import calc_coord
from datetime import datetime as dt
from astropy.time import Time

# import obs_log
ccd = ccd_old.ccd_controller()


# Info
# ----

name = "onepoint_track"
description = "Get optical onepoint tracking data"

# Default parameters
# ------------------

ra = 0.0
dec = 0.0
interval = 10.0
duration = 60.0

# Argument handler
# ================

p = argparse.ArgumentParser(description=description)
p.add_argument("--ra", type=float, help="Ra of target (degree).")
p.add_argument("--dec", type=float, help="Dec of target (degree).")
p.add_argument(
    "--interval", type=float, help="Interval time (sec). default=%.1f" % (interval)
)
p.add_argument(
    "--duration", type=float, help="Duration time (min). default=%.1f" % (duration)
)

args = p.parse_args()

if args.ra is not None:
    ra = args.ra
else:
    print("argument --ra is required")
    sys.exit()
if args.dec is not None:
    dec = args.dec
else:
    print("argument --dec is required")
    sys.exit()
if args.interval is not None:
    interval = args.interval
if args.duration is not None:
    duration = args.duration
if -360 <= ra <= 360.0 and -90 <= dec <= 90:
    pass
else:
    print("Please input 0~360 [deg]!!")
    sys.exit()

# Main
# ====

_list = []
_list.append("--ra")
_list.append(ra)
_list.append("--dec")
_list.append(dec)
_list.append("--interval")
_list.append(interval)
_list.append("--duration")
_list.append(duration)
# obs_log.start_script(name, _list)

tai_utc = (
    36.0  # tai_utc=TAI-UTC  2015 July from ftp://maia.usno.navy.mil/ser7/tai-utc.dat
)
dut1 = 0.14708

con = ROS_controller.controller()
target = "(ra,dec)=(" + str(ra) + "," + str(dec) + ")"
script = ""
for i in sys.argv:
    script += i
    script += " "
con.obs_status(
    active=True, obsmode="Onepoint_track", obs_script=script, obs_file="", target=target
)

con.dome_track()
coord = coord.coord_calc()
# ccd = ccd.ccd_client("172.20.0.12", 8010)

calc = calc_coord.azel_calc()


def handler(num, flame):
    con.move_stop()
    con.dome_stop()
    print("!!ctrl + c!!")
    print("Stop antenna")
    con.obs_status(active=False)
    time.sleep(2.0)
    sys.exit()


"""
def calc_star_azel(ra, dec, mjd):
    ra = ra*math.pi/180.
    dec = dec*math.pi/180.
    
    ret = slalib.sla_map(ra, dec, 0, 0, 0, 0, 2000, mjd + (tai_utc + 32.184)/(24.*3600.))
    ret = list(ret)
    ret = slalib.sla_aop(ret[0], ret[1], mjd, dut1, -67.70308139*math.pi/180, -22.96995611*math.pi/180, 4863.85, 0, 0, 283, 500, 0.1, 0.5, tlr=0.0065)
    real_az = ret[0]
    real_el = math.pi/2. - ret[1]
       
    real_az = real_az*180./math.pi
    real_el = real_el*180./math.pi
    ret = coord.apply_kisa_test(real_az, real_el, "hosei_opt.txt")
    real_az += ret[0]
    real_el += ret[1]
    return [real_az, real_el]
"""

now = dt.utcnow()
ret = calc.coordinate_calc(
    [ra * 3600], [dec * 3600], now, "fk5", 0, 0, "hosei_opt.txt", 0.5, 500, 300, 0.07
)

timestamp = time.strftime("%Y%m%d_%H%M%S")
signal.signal(signal.SIGINT, handler)
con.onepoint_move(ra, dec, "J2000", hosei="hosei_opt.txt", lamda=0.5)

con.antenna_tracking_check()
con.dome_tracking_check()

e_time = 0
error_count = 0

# start observation
while duration * 60 > e_time:
    tv = time.time()
    mjd = tv / 24.0 / 3600.0 + 40587.0
    status = con.read_status()
    # target = calc_star_azel(ra, dec, mjd)
    now = dt.utcnow()
    target = calc.coordinate_calc(
        [ra * 3600],
        [dec * 3600],
        Time(now),
        "fk5",
        0,
        0,
        "hosei_opt.txt",
        0.5,
        500,
        300,
        0.07,
    )
    try:
        ret = ccd.onepoint_shot(
            0, 0, target[0][0], target[1][0], "opt_track_" + timestamp, status
        )
        error_count = 0
    except Exception as e:
        print(e)
        error_count += 1
        if error_count > 3:
            con.move_stop()
            con.dome_stop()
            time.sleep(2.0)
            print("Program was stop in error.")
            sys.exit()
    if ret:
        print(ret)
        print("!!Can not find star!!")
    else:
        print("OK")
    tv2 = time.time()
    if tv2 - tv >= interval:
        e_time += tv2 - tv
    else:
        time.sleep(interval - (tv2 - tv))
        e_time += interval

con.move_stop()
con.dome_stop()
con.obs_status(active=False)
time.sleep(2.0)
print("Finish observation")

# obs_log.end_script(name)
