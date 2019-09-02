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
sys.path.append("/home/amigos/ros/src/nascorx_xffts")
import data_client

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

# main
con = ROS_controller.controller()
d = data_client.data_client()

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

for i in range(16):
    exec("d_hot_integ{} = []".format(i+1))
    exec("d_integ{} = []".format(i+1))
    exec("d_hot_sky{} = []".format(i+1))
    
#/SPEC_PMでtotal powerを受け取る場合スプリアスが考慮されない
#d_hot_raw = con.oneshot_achilles(repeat = 1, exposure = 1.0, stime = 1.0)
data = d.oneshot(exposure, 1 ,time.time()+1)[2][0]
for i in range(16):
    exec("d_hot_integ{}_tmp = np.sum(data[{}])".format(i+1, i))
    exec("d_hot_integ{}.append(d_hot_integ{}_tmp)".format(i+1, i+1))

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
    data = d.oneshot(exposure, 1 ,time.time()+1)[2][0]
    for i in range(16):
        exec("d_hot_integ{}_tmp = np.sum(data[{}])".format(i+1, i))
        exec("d_integ{}.append(d_hot_integ{}_tmp)".format(i+1, i+1))

#解析開始
#log(p_hot-p_sky)のlistの作成

for i in range(16):
    for j in range(len(d_integ1)):
        d_temp = eval("d_hot_integ{}[0]-d_integ{}[j]".format(i+1, i+1))
        d_temp2 = math.log(d_temp)
        exec("d_hot_sky{} = d_hot_sky{} + [d_temp2]".format(i+1, i+1))

        
# seczの計算
secz = []
for i in range(len(z)):
    secz_temp = (z[i]/180)*math.pi
    secz.append(1/math.cos(secz_temp))

print(d_hot_sky1)
print(d_hot_sky2)
#plot
for i in range(6):
    fig, (axL, axR) = plt.subplots(ncols = 2, figsize = (10,4))

    #fitting
    fit_array1 = np.polyfit(secz, eval("d_hot_sky{}".format(1+i*2)),1)
    fit_array2 = np.polyfit(secz, eval("d_hot_sky{}".format(2+i*2)),1)
    
    axL.plot(secz, np.poly1d(fit_array1)(secz), label="dfs1")
    axL.plot(secz, eval("d_hot_sky{}".format(1+i*2)), 'o')
    axL.set_title('IF{}'.format(1+i*2))
    axL.set_xlabel('secz')
    axL.set_ylabel('Power')

    axR.plot(secz, np.poly1d(fit_array2)(secz), label="dfs1")
    axR.plot(secz, eval("d_hot_sky{}".format(2+i*2)), 'o')
    axR.set_title('IF{}'.format(2+i*2))
    axR.set_xlabel('secz')
    axR.set_ylabel('Power')

    axR.text(0.1, 0.1, 'tau = {:.4f}'.format(fit_array1[0]), transform = axL.transAxes)
    axR.text(1.4, 0.1, 'tau = {:.4f}'.format(fit_array2[0]) , transform = axL.transAxes)

    axL.grid()
    axR.grid()
    plt.tight_layout()
    plt.savefig(os.path.join(data_dir, "IF{}_{}.png".format(1+2*i, 2+2*i)))
    plt.show()

log.info("Observation END")
