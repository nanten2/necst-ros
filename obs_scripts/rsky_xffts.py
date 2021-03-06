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
import logger

import sys
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")
sys.path.append("/home/amigos/ros/src/nascorx_xffts/")
import ROS_controller


#setup logger
#===========
now = datetime.datetime.utcnow()
log_path = '/home/amigos/log/{}.txt'.format(now.strftime('%Y%m%d'))
logger = logger.logger(__name__, filename=log_path)
log = logger.setup_logger()
logger.obslog(sys.argv)
start_time = time.time()

import signal
def handler(num, flame):
    print("!!ctrl+C!!")
    print("STOP MOVING")
    con.move_stop()
    con.dome_stop()
    con.pub_loggerflag("")
    time.sleep(1)#?
    logger.obslog("STOP OBSERVATION", lv=1)
    con.obs_status(active=False)
    sys.exit()
signal.signal(signal.SIGINT, handler)
                                
con = ROS_controller.controller()

# Initial configurations
# ----------------------

timestamp = time.strftime('%Y%m%d_%H%M%S')
if memo != '':
    name = name + '_' + memo
else:
    pass
#savedir = "/home/amigos/data/rsky/"+timestamp+"/"
savedir = "./observation/rsky/"+timestamp
savedir2 = "/home/amigos/hdd/data/observation/rsky/"+timestamp
if not os.path.exists(savedir2):
    os.makedirs(savedir2)
else:
    pass
savepath = os.path.join(savedir, "xffts.ndf")

# Data aquisition
# ---------------

#status = con.read_status()
#cabin_temp = status.CabinTemp1
#if cabin_temp < 10.: # if no data
#    cabin_temp = 300
#    print("No data from weather")
#    print("temporary T=300[K]")
#else:
#    pass

print('Start experimentation')

print('')

print('R')
#con.move_hot('in')
con.move_chopper("in")
time.sleep(3)# Temporarily

status = con.read_status()
hot_status = status.Current_Hot
print('hot_status ### ', hot_status)
#print('cabin_temp: %.2f'%(cabin_temp))

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

print('get spectrum...')
con.xffts_publish_flag(obs_mode="SKY")
time.sleep(integ)
con.xffts_publish_flag()

logger.obslog('Observation End : observation time : {:.2f} [min]'.format((time.time() - start_time)/60), lv=1)
log.info('Observation End : observation time : {:.2f} [min]'.format((time.time() - start_time)/60))

#con.move_hot('in')
con.pub_loggerflag("")
con.pub_analyexec(savedir2, "rsky")

