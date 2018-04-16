#! /usr/bin/env python

import time
import os
import sys
import argparse
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")
import ROS_controller
sys.path.append("/home/amigos/ros/src/necst/lib")
import obs_log



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
if args.snow is not None: snow = args.snow

# Main
# ====

list = []
if snow:
    list.append("--snow")
    list.append(snow)
obs_log.start_script(name, list)

ctrl = ROS_controller.controller()
time.sleep(0.5)
try:
    ctrl.move_stop()
except:
    print("Already tracking_end.")
try:
    ctrl.dome_stop()
except:
    print("Already dome_track_end.")

#status = ctrl.read_status()
#if status["Drive_ready_Az"] == "ON" and status["Drive_ready_El"] == "ON":
#print("antenna_move")
time.sleep(0.3)
if snow:
    ctrl.move(-90, 0.0, limit=False)
else:
    ctrl.move(0, 45)
ctrl.antenna_tracking_check()
print("memb_close")
ctrl.memb_close()

time.sleep(1.5)
print("dome_close")
ctrl.dome_close()
time.sleep(1.5)

print("dome_move")
ctrl.dome_move(90)
ctrl.dome_tracking_check()
ctrl.drive("off")
print("End observation")
obs_log.end_script(name)
    

