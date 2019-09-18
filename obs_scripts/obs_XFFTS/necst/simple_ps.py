#! /usr/bin/env python3

# Configurations
# ==============

# Info
# ----

name = 'rsky_by_spectrometer'
description = 'Get R-Sky data.'


# Default parameters
# ------------------

integ = 1
memo = ''

# object parameters
# =================
# test
#on_ra = 160
#on_dec = -3
#off_ra = on_ra+2
#off_dec = on_dec

# M42
#on_ra = 83.8221
#on_dec = -05.3911
#off_ra = on_ra+1.5
#off_dec = on_dec

# # W51 IRS 2
# #ICRS coord. (ep=J2000) :19 23 40.05 +14 31 07.1 
on_ra = 290.958
on_dec = 14.1
off_ra = on_ra+1.5
off_dec = on_dec

# # W 49 -- Star forming region
# #ICRS coord. (ep=J2000) :19 10 19.6 +09 07 42
# on_ra = 287.581
# on_dec = 9.128
# off_ra = on_ra+1.5
# off_dec = on_dec

# # W43 SNR G030.8-00.0 -- SuperNova Remnant
# #ICRS coord. (ep=J2000) :18 47 32.4 -01 56 31
# on_ra = 281.885
# on_dec = -1.942
# off_ra = on_ra+1.5
# off_dec = on_dec


# Argument handler
# ================

import argparse

p = argparse.ArgumentParser(description=description)
p.add_argument('--integ', type=float,
               help='Integration time (sec). default=%.1f'%(integ))
p.add_argument('--memo', type=str,
               help='Working memo. This will be include in directroy name.')

args = p.parse_args()

if args.integ is not None: integ = args.integ
if args.memo is not None: memo = args.memo


# Main
# ====

import os
import time
import numpy
import datetime

import sys
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")
sys.path.append("/home/amigos/ros/src/nascorx_xffts/")
import ROS_controller
import data_client
d = data_client.data_client()

# Initial configurations
# ----------------------

timestamp = time.strftime('%Y%m%d_%H%M%S')
if memo != '':
    name = name + '_' + memo
else:
    pass
#savedir = "/home/amigos/data/rsky/"+timestamp+"/"
savedir = "./observation/simple_ps/"+timestamp
savedir2 = "/home/amigos/hdd/data/observation/simple_ps/"+timestamp
if not os.path.exists(savedir2):
    os.makedirs(savedir2)
else:
    pass
savepath = os.path.join(savedir, "xffts.ndf")

# Data aquisition
# ---------------

con = ROS_controller.controller()
con.dome_track()

status = con.read_status()
cabin_temp = status.CabinTemp1
if cabin_temp < 10.: # if no data
    cabin_temp = 300
    print("No data from weather")
    print("temporary T=300[K]")
else:
    pass

print('Start experimentation')

# HOT
# ===
print('R')
#con.move_hot('in')
con.move_chopper("in")
time.sleep(3)# Temporarily

status = con.read_status()
hot_status = status.Current_Hot
print('hot_status ### ', hot_status)
print('cabin_temp: %.2f'%(cabin_temp))
print('get spectrum...')
con.pub_loggerflag(savedir)
con.xffts_publish_flag(obs_mode="HOT")
time.sleep(integ)
con.xffts_publish_flag()

print('SKY')
con.move_chopper("out")
time.sleep(3)# Temporarily

status = con.read_status()
hot_status = status.Current_Hot
print('hot_status ### ', hot_status)

# ON
# ===
print('get spectrum...')
con.onepoint_move(on_ra, on_dec, "fk5")
con.antenna_tracking_check()
con.dome_tracking_check()
con.xffts_publish_flag(obs_mode="ON")
time.sleep(integ)
con.xffts_publish_flag()

# OFF
# ===
con.onepoint_move(off_ra, off_dec, "fk5")
con.antenna_tracking_check()
con.dome_tracking_check()
con.xffts_publish_flag(obs_mode="OFF")
time.sleep(integ)
con.xffts_publish_flag()

con.move_stop()
con.pub_loggerflag("")
con.pub_analyexec(savedir2, "simple_ps")
time.sleep(2)
