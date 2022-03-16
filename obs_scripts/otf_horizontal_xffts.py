#! /usr/bin/env python3
# coding:utf-8

import os
import shutil
import numpy
from datetime import datetime, timedelta
import sys
import argparse
import logger
import time
import read_obsfile
import signal
import ROS_controller
import doppler_nanten

# Configurations
# ==============
name = "otf_2019"
description = "Get OTF spectrum"

# Default parameters
# ==================
obsfile = ""
tau = 0.0
c = 299792458


# setup logger
# ===========
now = datetime.utcnow()
log_path = "/home/amigos/log/{}.txt".format(now.strftime("%Y%m%d"))
logger = logger.logger(__name__, filename=log_path)
log = logger.setup_logger()
logger.obslog(sys.argv)
start_time = time.time()

# Read Observation file
# =====================
# Argument handler
# ================
obsdir = "/home/amigos/necst-obsfiles/"

p = argparse.ArgumentParser(description=description)
p.add_argument("--obsfile", type=str, help="absolute path for obsfile")
p.add_argument("--tau", type=float, help="tau. default=%.1f" % (tau))

args = p.parse_args()

if args.obsfile is not None:
    obsfile = args.obsfile
if args.tau is not None:
    tau = args.tau
try:
    obs = read_obsfile.read(os.path.join(obsdir, obsfile))
except Exception as e:
    log.exception(e)
    sys.exit()

# Save file
# =========
datahome = "./observation/otf"
timestamp = time.strftime("%Y%m%d%H%M%S")
dirname = "n%s_%s_%s_otf_%s" % (
    timestamp,
    obs["molecule_1"],
    obs["transiti_1"].split("=")[1],
    obs["object"],
)
savedir = os.path.join(datahome, name, dirname)
log.info("mkdir {savedir}".format(**locals()))
os.makedirs(savedir)
logger.obslog("savedir : {}".format(savedir), lv=1)

dirname = "n{}_{}_{}_horizontalotf_{}".format(
    now.strftime("%Y%m%d%H%M%S"),
    obs["molecule_1"],
    obs["transiti_1"].split("=")[1],
    obs["object"],
)
xffts_datapath = os.path.join(savedir, "xffts.ndf")

log.debug("obsdir : {}".format(obsdir))
log.debug("log_path : {}".format(log_path))
log.debug("dirname : {}".format(dirname))
log.debug("xffts : {}".format(xffts_datapath))

# Main
# ====
dp = doppler_nanten.doppler_nanten()
con = ROS_controller.controller()
con.pub_loggerflag(savedir)  # logger start
con.dome_track()
con.move_stop()
con.pub_encdb_flag(True, os.path.join(savedir, "enc.dat"))

# copy hosei & obsfiles
# =====================
con.pub_txtfile(
    "/home/amigos/ros/src/necst/lib/hosei_230.txt",
    os.path.join(savedir, "hosei_230.txt"),
)
con.pub_txtfile(os.path.join(obsdir, obsfile), os.path.join(savedir, obsfile))


def handler(num, flame):
    log.warn("!!ctrl+C!!")
    log.warn("STOP MOVING")
    con.move_stop()
    con.dome_stop()
    con.obs_status(active=False)
    con.xffts_publish_flag(obs_mode="", scan_nun=num)
    con.pub_loggerflag("")
    time.sleep(2)
    logger.obslog("STOP OBSERVATION", lv=1)
    time.sleep(1.0)
    sys.exit()


signal.signal(signal.SIGINT, handler)

# param
# =====
lambda_on = obs["lambda_on"]  # on
beta_on = obs["beta_on"]  # on
lambda_off = obs["lambda_off"]  # off
beta_off = obs["beta_off"]  # off
coordsys = obs["coordsys"].lower()  # coord
dcos = obs["otadel"]  # dcos
integ_on = obs["exposure"]  # on iinteg
integ_off = obs["exposure_off"]  # off integ

# lamdel_on = obs['lamdel']# offset_on
# betdel_on = obs['betdel']# offset_on
lamdel_off = obs["lamdel_off"]  # offset_off
betdel_off = obs["betdel_off"]  # offset_off
cosydel = obs["cosydel"].lower()  # offset_coord
offset_dcos = obs["otadel_off"]  # offset_dcos
vlsr = obs["vlsr"]

# Scan Parameters
# ---------------

sx = obs["lamdel_off"] + obs["start_pos_x"]  # [arcsec]
sy = obs["betdel_off"] + obs["start_pos_y"]  # [arcsec]
direction = int(obs["scan_direction"])

if direction == 0:
    dx = float(obs["otfvel"]) * float(obs["exposure"])  # [arcsec]
    dy = 0
    gridx = 0
    gridy = obs["grid"]  # [arcsec]
elif direction == 1:
    dx = 0
    dy = float(obs["otfvel"]) * float(obs["exposure"])  # [arcsec]
    gridx = obs["grid"]  # [arcsec]
    gridy = 0
else:
    con.move_stop()
    print("Error:direction")
    sys.exit()
dt = obs["exposure"]  # float(obs['grid']/obs['otfvel'])

if obs["otadel"].lower() == "y":
    offset_dcos = 1
    dcos = 1
else:
    offset_dcos = 0
    dcos = 0

if obs["lo1st_sb_1"] == "U":  # 後半に似たのがあるけど気にしない
    sb1 = 1
else:
    sb1 = -1
if obs["lo1st_sb_2"] == "U":  # 後半に似たのがあるけど気にしない
    sb2 = 1
else:
    sb2 = -1


lamda = c / float(obs["restfreq_1"])  # 分光計 1
otflen = obs["otflen"] / obs["exposure"]
scan_point = float(otflen)  # scan_point for 1 line
scan_point = int(scan_point)
rampt = dt * obs["lamp_pixels"]
log.info("scan point is {}".format(scan_point))

total_count = int(obs["N"])  # total scan_line

log.info("Start Observation")

status = con.read_status()
savetime = status.Time


rp = int(obs["nTest"])
rp_num = 0
n = int(total_count)
latest_hottime = 0

# start observation
# ---------------
obsscript = __file__
if obs["scan_direction"] == 0:
    con.obs_status(
        True,
        obs["obsmode"],
        obsscript,
        obsfile,
        obs["object"],
        scan_point,
        total_count,
        dx,
        gridy,
        integ_off,
        integ_off,
        integ_on,
        "x",
    )
else:
    con.obs_status(
        True,
        obs["obsmode"],
        obsscript,
        obsfile,
        obs["object"],
        scan_point,
        total_count,
        gridx,
        dy,
        integ_off,
        integ_off,
        integ_on,
        "y",
    )

while rp_num < rp:
    log.info("repeat : {}".format(rp_num))
    num = 0
    while num < n:
        log.info("observation : {}".format(num))
        log.info("tracking start")
        con.move_stop()

        con.onepoint_move(
            lambda_off,
            beta_off,
            coordsys,
            off_x=lamdel_off,
            off_y=betdel_off,
            offcoord=cosydel,
            dcos=dcos,
        )

        log.info("check_track")
        con.antenna_tracking_check()
        con.dome_tracking_check()
        log.info("tracking OK")

        _now = time.time()
        if _now > latest_hottime + 60 * obs["load_interval"]:
            log.info("R")
            con.move_chopper("in")
            time.sleep(3)
            con.obs_status(
                active=True, current_num=scan_point * num, current_position="HOT"
            )

            print("get spectrum...")
            log.info("cosydel {}".format(cosydel))

            status = con.read_status()
            temp = float(status.CabinTemp1)
            con.xffts_publish_flag(scan_num=num, obs_mode="HOT")
            time.sleep(integ_off)
            con.xffts_publish_flag(scan_num=num, obs_mode="")
            latest_hottime = time.time()
            pass

        else:
            pass

        log.info("OFF")
        con.move_chopper("out")
        time.sleep(3)
        con.obs_status(
            active=True, current_num=scan_point * num, current_position="OFF"
        )
        log.info("get spectrum...")

        status = con.read_status()
        temp = float(status.CabinTemp1)
        con.xffts_publish_flag(scan_num=num, obs_mode="OFF")
        time.sleep(integ_off)
        con.xffts_publish_flag(scan_num=num, obs_mode="")
        log.info("move ON")
        con.move_stop()
        ssx = (
            (sx + num * gridx) - float(dx) / float(dt) * rampt - float(dx) / 2.0
        )  # rampの始まり
        ssy = (
            (sy + num * gridy) - float(dy) / float(dt) * rampt - float(dy) / 2.0
        )  # rampの始まり

        log.info("ramp_start tracking")
        con.onepoint_move(
            lambda_on,
            beta_on,
            coordsys,
            off_x=ssx,
            off_y=ssy,
            offcoord=cosydel,
            dcos=dcos,
        )
        con.antenna_tracking_check()
        con.dome_tracking_check()

        log.info("reach ramp_start")  # rampまで移動

        log.info(" OTF scan_start!! ")
        log.info("move ON")
        delay = 3.0
        ctime = time.time()
        start_on = 40587 + (ctime + rampt + delay) / 24.0 / 3600.0  # mjd
        con.obs_status(active=True, current_num=scan_point * num, current_position="ON")
        # con.horizontal_scan(lambda_on, beta_on, coordsys, dx, dy, dt, scan_point, rampt, delay=delay, current_time=ctime, off_x = sx + num*gridx, off_y = sy + num*gridy, offcoord = cosydel, dcos=dcos, hosei='hosei_230.txt', lamda=lamda, limit=True)
        con.horizontal_scan(
            lambda_on,
            beta_on,
            coordsys,
            dx,
            dy,
            dt,
            scan_point,
            rampt,
            delay=delay,
            current_time=ctime,
            off_x=sx + num * gridx,
            off_y=sy + num * gridy,
            offcoord=cosydel,
            dcos=dcos,
            hosei="hosei_230.txt",
            lamda=lamda,
            limit=True,
        )
        log.info("getting_data...")
        con.xffts_publish_flag(scan_num=num, obs_mode="ON")
        log.info("start_on : {}".format(start_on))
        while start_on + (
            obs["otflen"] + rampt
        ) / 24.0 / 3600.0 > 40587 + time.time() / (24.0 * 3600.0):
            # while obs['otflen']/24./3600. > 40587 + time.time()/(24.*3600.):
            time.sleep(0.001)
        con.xffts_publish_flag(obs_mode="", scan_num=num)

        _on = 0
        while _on < scan_point:
            log.info("{}".format(_on + 1))
            _on += 1

        log.info("stop")
        con.move_stop()
        num += 1
        continue

    rp_num += 1
    continue

log.info("R")  # 最初と最後をhotではさむ
# con.move_hot('in')
con.move_chopper("in")
time.sleep(3)
con.obs_status(active=True, current_num=scan_point * num + 1, current_position="HOT")

con.xffts_publish_flag(scan_num=num, obs_mode="HOT")
time.sleep(integ_off)
con.xffts_publish_flag(scan_num=num, obs_mode="")

con.move_chopper("out")
time.sleep(3)

logger.obslog(
    "Observation End : observation time : {:.2f} [min]".format(
        (time.time() - start_time) / 60
    ),
    lv=1,
)
log.info(
    "Observation End : observation time : {:.2f} [min]".format(
        (time.time() - start_time) / 60
    )
)
con.move_stop()
con.dome_stop()
con.pub_loggerflag("")
time.sleep(2)
