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
    print("*** system stop!! ***")
    ctrl.move_stop()
    ctrl.obs_status(active=False)
    sys.exit()
    return
signal.signal(signal.SIGINT, handler)

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

if args.opt is not None:
    opt = args.opt
    target = "initialize (opt)"
else:
    target = "initialize (nomal)"
# Main
# ====

ctrl = ROS_controller.controller()
ctrl.obs_status(active=True, obsmode="INITIALIZE", obs_script=__file__, obs_file="no file", target = target)
ctrl.drive("on")
ctrl.obs_status(active=True, current_position="ok : drive on")
print("dome_open")
ctrl.dome_open()
ctrl.obs_status(active=True, current_position="ok : dome open")

time.sleep(2.)
if opt:
    print("memb_open")
    ctrl.memb_open()
    ctrl.obs_status(active=True, current_position="ok : membrane open")    
    time.sleep(2.)

print("Init end")
ctrl.dome_track()
ctrl.dome_tracking_check()
time.sleep(5.)
ctrl.dome_track_end()
ctrl.obs_status(active=True, current_position="ok : dome track")
time.sleep(2.)
ctrl.obs_status(active=False)    
time.sleep(2.)
