#! /usr/bin/env python3
# coding:utf-8


# Configurations
# ==============
# Info
# ----

name = "opt_cross_pointing_line_9"
description = "Do radio pointing"


# Default parameters
# ------------------
planet = ""
# Argument handler
# ================

import argparse
import sys

p = argparse.ArgumentParser(description=description)
p.add_argument("--planet", type=str, help="target planet name", required=True)
args = p.parse_args()
if args.planet is not None:
    planet = args.planet

planet_list = {
    "MERCURY": 1,
    "VENUS": 2,
    "MARS": 4,
    "JUPITER": 5,
    "SATURN": 6,
    "URANUS": 7,
    "NEPTUNE": 8,
    "MOON": 10,
    "SUN": 11,
}
if planet.upper() in planet_list:
    number = planet_list[planet.upper()]
else:
    print("undefined planet name")
    sys.exit()
# Main
# ====
import os
from datetime import datetime as dt
import time
import signal
import numpy
from astropy.coordinates import SkyCoord, EarthLocation, AltAz, get_body
from astropy.time import Time
import astropy.units as u

nanten2 = EarthLocation(
    lat=-22.96995611 * u.deg, lon=-67.70308139 * u.deg, height=4863.85 * u.m
)

import ROS_controller

con = ROS_controller.controller()
con.dome_track()
import ccd_old

ccd = ccd_old.ccd_controller()


def handler(num, flame):
    con.move_stop()
    con.dome_stop()
    print("!!ctrl + c!!")
    print("Stop antenna")
    con.obs_status(active=False)
    sys.exit()


signal.signal(signal.SIGINT, handler)

# Initial configurations
# ----------------------

now = dt.utcnow()
timestamp = now.strftime("opt_%Y%m%d%H%M%S")
dirname = "/home/amigos/data/experiment/opt_cross_point/"
savedir = os.path.join(dirname, timestamp)

print("mkdir {savedir}".format(**locals()))
os.makedirs(savedir)

# Data aquisition
# ---------------
point_n = 5  # 2/1line
obs_n = 2  # 2*run_nember
xgrid = 30
ygrid = 30
num = 0

con.obs_status(
    active=True,
    obsmode="Opt_cross",
    obs_script=__file__,
    obs_file="star",
    target=planet,
)
while num < obs_n:
    p_n = 0
    while p_n < point_n:
        off_x = 0
        off_y = 0

        print("num " + str(num))
        print("p_n " + str(p_n))

        if num % 2 == 0:
            off_x = xgrid * (p_n - (int(point_n / 2)))
        else:
            off_y = ygrid * (p_n - (int(point_n / 2)))

        print("observation :" + str(num))
        print("tracking start")
        con.move_stop()
        con.planet_move(number, off_x=off_x, off_y=off_y, offcoord="altaz", dcos=1)
        print("moving...")
        con.antenna_tracking_check()
        con.dome_tracking_check()
        print("tracking OK")
        con.obs_status(
            active=True, current_num=num * obs_n + p_n, current_position="ON"
        )
        print("get spectrum...")
        ctime = dt.utcnow()
        savedata = ctime.strftime("%Y%m%d_%H%M%S")
        con.ccd_oneshot(savedata, savedir)

        status = con.read_status()
        coord = get_body(planet, Time(ctime), location=nanten2)
        coord.obswl = 0.5 * u.mm
        coord.pressure = status.Press * u.hPa
        coord.temperature = status.OutTemp * u.deg_C
        coord.relative_humidity = status.OutHumi / 100
        coord.location = nanten2
        azel = coord.transform_to(AltAz(obstime=Time(now)))
        az = azel.az.arcsec
        el = azel.alt.arcsec
        xx, yy = ccd.ccd_analysis(savedata, savedir)
        ccd.save_status(xx, yy, 0, status.Secofday, az, el, savedir, savedata, status)
        print("stop")
        con.move_stop()

        p_n += 1
    num += 1

import opt_analy

opt_analy.opt_plot(
    [savedir], savefig=True, figname=savedir.split("/")[-2], interactive=True
)
con.obs_status(active=False)
print("end_observation")
