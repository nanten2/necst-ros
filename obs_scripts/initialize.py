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

name = 'initialize'
description = 'Initialize antenna'

# Default parameters
# ------------------

opt = ''

# Argument handler
# ================

p = argparse.ArgumentParser(description=description)
p.add_argument('--opt', type=str,
               help='For optical. Need 1.')

args = p.parse_args()

if args.opt is not None: opt = args.opt

# Main
# ====

obs_log.start_script(name)
obs_log.weather_log()

ctrl = ROS_controller.controller()
time.sleep(0.5)
ctrl.drive_on()
ctrl.contactor_on()
print("dome_open")
ctrl.dome_open()

if opt:
    print("memb_open")
    ctrl.memb_open()
print("Init end")
ctrl.dome_track()
time.sleep(0.8)
ctrl.dome_track_end()

obs_log.end_script(name)
