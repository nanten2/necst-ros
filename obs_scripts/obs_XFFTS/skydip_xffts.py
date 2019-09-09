#!/usr/bin/env python
# coding:utf-8

import ROS_controller
import time
import math
import matplotlib.pylab as plt
from scipy.optimize import curve_fit
import numpy as np
import sys
import signal
import logger
import os
from datetime import datetime

# Info
description = 'Get R-Sky data.'

# Defalut Parameter
exposure = 1.0 #[sec]

# Argument handler
# ================
import argparse

p = argparse.ArgumentParser(description=description)
p.add_argument('--integ', type=float,
                              help='Integration time (sec). default=%.1f'%(exposure))

args = p.parse_args()

if args.integ is not None: exposure = args.integ

# Setup logging
now = datetime.utcnow()
logdir = "/home/amigos/log"
if not os.path.exists(logdir):
    os.mkdir(logdir)
log_path = '/home/amigos/log/{}.log'.format(now.strftime('%Y%m%d'))
logger = logger.logger(__name__, filename=log_path)
log = logger.setup_logger()

# data path
data_dir = "/home/amigos/data/skydip_xffts/{}".format(now.strftime("%Y%m%d%H%M%S"))
if not os.path.exists(data_dir):
    os.makedirs(data_dir)
savepath = os.path.join(data_dir, "skydip.ndf")

# main
con = ROS_controller.controller()

# Ctrl+Cで中止するコマンド
def handler(num, flame):
    con.move_stop()
    log.warn("!!Ctrl + C!!")
    log.warn("Stop anntena")
    con.obs_status(active = False)
    sys.exit()

signal.signal(signal.SIGINT, handler)
    
opt = con.read_status()

#HOT点の観測
log.info("observation : HOT")
while not opt.Current_Hot == 'IN':
    opt = con.read_status()
    time.sleep(1)
    con.move_hot('IN')

#/SPEC_PMでtotal powerを受け取る場合スプリアスが考慮されない
#d_hot_raw = con.oneshot_achilles(repeat = 1, exposure = 1.0, stime = 1.0)
con.xffts_publish_flag(1, savepath, obs_mode="HOT")
time.sleep(exposure)
con.xffts_publish_flag(0, savepath, obs_mode="HOT")

while not opt.Current_Hot == 'OUT':
    opt = con.read_status()
    time.sleep(1)
    con.move_hot('OUT')

opt = con.read_status()

#skydip測定の開始
z = [80, 70, 60, 45, 30, 25, 20]

con.dome_track()
log.info("Observation : Sky")
for elevation in z:
    log.info("Elevation : {}".format(elevation))
    con.onepoint_move(30,elevation)
    con.dome_tracking_check()
    log.info("dome track OK")
    con.antenna_tracking_check()
    log.info("antenna track OK")
    con.move_stop()
    con.xffts_publish_flag(1, savepath, obs_mode="SKY", scan_num=elevation)
    time.sleep(exposure)
    con.xffts_publish_flag(0, savepath, obs_mode="SKY", scan_num=elevation)

con.pub_analyexec(data_dir, "skydip")
