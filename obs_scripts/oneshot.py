#! /usr/bin/env python2

import time
import os
import sys
import argparse
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")
import ROS_controller
import ccd
import signal
ccd = ccd.ccd_controller()

# Info
# ----

name = 'oneshot'
description = 'Get oneshot data'

# Default parameters
# ------------------

star = ''
filename = ''

# Argument handler
# ================

p = argparse.ArgumentParser(description=description)
p.add_argument('--star', type=str,
               help='Name of 1st magnitude star.(No space)')
p.add_argument('--name', type=str,
               help='save file name')

args = p.parse_args()

if args.star is None:
    print('!!Star name is None!!')
    sys.exit()
else:
    star = args.star
if args.name is not None: filename = args.name


# Main
# ====

#timestamp = time.strftime('%Y%m%d_%H%M%S') #time.strftime("%Y%m%d_%H%M%S", time.gmtime())
#timestamp = timestamp+memo
star_list = []
planet_list = {"MERCURY":1, "VENUS":2, "MARS":4, "JUPITER":5, "SATURN":6, "URANUS":7, "NEPTUNE":8, "MOON":10, "SUN":11}
planet = 0
target = []

#read star list
try:
    f = open("/home/amigos/ros/src/necst/lib/1st_star_list.txt")
except:
    f = open("/home/necst/ros/src/necst/lib/1st_star_list.txt")
line = f.readline()
while line:
    line = line.split()
    star_list.append(line)
    line = f.readline()

for i in range(len(star_list)):
    if star_list[i][0].upper() == star.upper():
        target.append(float(star_list[i][1]))
        target.append(float(star_list[i][2]))

if len(target) == 0:
    if star.upper() in planet_list:
        planet = planet_list[star.upper()]
    else:
        print('!!Can not find the name of star!!')
        sys.exit()

ctrl = ROS_controller.controller()

def handler(num, flame):
    ctrl.move_stop()
    print("!!ctrl + c!!")
    print("Stop antenna")
    sys.exit()


signal.signal(signal.SIGINT, handler)

ctrl.dome_track()
ctrl.move_stop()

if planet:
    ctrl.planet_move(planet, hosei = "hosei_opt.txt", lamda = 0.5)
else:
    ctrl.radec_move(target[0], target[1], 'J2000', 0, 0, offcoord="HORIZONTAL", hosei='hosei_opt.txt', lamda = 0.5)

b_az = 0
ctrl.dome_tracking_check()#test
ctrl.antenna_tracking_check()#test

if not filename:
    filename = time.strftime("%H%M%S")
dirname = time.strftime("%Y%m%d")
ccd.oneshot(dirname, filename)
print(dirname, filename)

ctrl.dome_track_end()#test
ctrl.move_stop()#test
print("Finish observation")

