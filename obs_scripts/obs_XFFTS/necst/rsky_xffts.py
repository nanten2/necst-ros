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
savedir = "/home/amigos/hdd/data/observation/rsky/"+timestamp
if not os.path.exists(savedir):
    os.makedirs(savedir)
else:
    pass
savepath = os.path.join(savedir, "xffts.ndf")

# Data aquisition
# ---------------

con = ROS_controller.controller()
status = con.read_status()
cabin_temp = status.CabinTemp1
if cabin_temp < 10.: # if no data
    cabin_temp = 300
    print("No data from weather")
    print("temporary T=300[K]")
else:
    pass

print('Start experimentation')

print('')

print('R')
#con.move_hot('in')
con.move_chopper("in")
time.sleep(3)# Temporarily

status = con.read_status()
hot_status = status.Current_Hot
print('hot_status ### ', hot_status)
# while True:
#     status = con.read_status()
#     hot_status = status.Current_Hot
#     print('hot_status ### ',hot_status)
#     if not hot_status == 'IN':
#         time.sleep(0.5)
#         continue
#     else:
#         break

print('cabin_temp: %.2f'%(cabin_temp))

print('get spectrum...')
con.pub_loggerflag(savedir)
con.xffts_publish_flag(obs_mode="HOT")
time.sleep(integ)
#con.pub_loggerflag("")
con.xffts_publish_flag()

print('SKY')
#con.move_hot('out')
con.move_chopper("out")
time.sleep(3)# Temporarily

status = con.read_status()
hot_status = status.Current_Hot

print('hot_status ### ', hot_status)

# while True:
#     status = con.read_status()
#     hot_status = status.Current_Hot
#     if not hot_status == 'OUT':
#         time.sleep(0.5)
#         continue
#     else:
#         break


print('get spectrum...')
con.xffts_publish_flag(obs_mode="SKY")
time.sleep(integ)
con.xffts_publish_flag()


#con.move_hot('in')
con.pub_loggerflag("")
con.pub_analyexec(savedir, "rsky")

