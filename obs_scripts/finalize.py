#! /usr/bin/env python

import time
import os
import sys
import argparse
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")
import ROS_controller
sys.path.append("/home/amigos/ros/src/necst/lib")
import signal
def handler(signal, frame):
    ctrl.move_stop()
    print("*** system stop!! ***")
    ctrl.move_stop()
    ctrl.obs_status(active=False)
    time.sleep(2.)
    sys.exit()
    return

signal.signal(signal.SIGINT, handler)


# Info
# ----

name = 'finalize'
description = 'Finalize observation'

# Default parameters
# ------------------

snow = ''

# Argument handler
# ================

p = argparse.ArgumentParser(description=description)
p.add_argument('--snow', type=str,
               help='For snow position. Need 1.')
args = p.parse_args()
if args.snow is not None:
    snow = args.snow
    target = "finalize (snow)"
else:
    target = "finalize (normal)"

# Main
# ====

ctrl = ROS_controller.controller()
ctrl.obs_status(active=True, obsmode="FINALIZE", obs_script=__file__, obs_file="no file", target=target)

time.sleep(0.5)
try:
    ctrl.move_stop()
    ctrl.obs_status(active=True, current_position="ok : move stop")
except:
    print("Already tracking_end.")
try:
    ctrl.dome_stop()
    ctrl.obs_status(active=True, current_position="ok : dome stop")    
except:
    print("Already dome_track_end.")

#status = ctrl.read_status()
#if status["Drive_ready_Az"] == "ON" and status["Drive_ready_El"] == "ON":
#print("antenna_move")
time.sleep(0.3)
if snow:
    ctrl.onepoint_move(-90, 0.0001, limit=False)
else:
    ctrl.onepoint_move(0, 45)
time.sleep(2.)
ctrl.antenna_tracking_check()
ctrl.obs_status(active=True, current_position="ok : home position")
print("memb_close")
ctrl.memb_close()
ctrl.obs_status(active=True, current_position="ok : membrane close")

time.sleep(1.5)
print("dome_close")
ctrl.dome_close()
ctrl.obs_status(active=True, current_position="ok : dome close")
time.sleep(1.5)

print("dome_move")
ctrl.dome_move(90)
ctrl.obs_status(active=True, current_position="ok : dome move")
time.sleep(2.)
ctrl.drive("off")
ctrl.obs_status(active=True, current_position="ok : drive off")

print("End observation")
time.sleep(2.)
ctrl.obs_status(active=False)
time.sleep(2.)
