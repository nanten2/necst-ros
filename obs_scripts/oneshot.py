#! /usr/bin/env python3

import time
import os
import sys
import argparse
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")
sys.path.append("/home/amigos/ros/src/necst/scripts/device")
import ROS_controller
from astropy.coordinates import get_body,EarthLocation, SkyCoord
from datetime import datetime as dt
from astropy.time import Time
import astropy.units as u
nanten2 = EarthLocation(lat=-22.9699511*u.deg, lon=-67.60308139*u.deg, height=4863.84*u.m)

import signal
def handler(num, flame):
    ctrl.move_stop()
    ctrl.obs_status(active=False)
    print("!!ctrl + c!!")
    print("Stop antenna")
    sys.exit()
signal.signal(signal.SIGINT, handler)


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
planet_list = {"MERCURY":1, "VENUS":2, "MARS":4, "JUPITER":5, "SATURN":6, "URANUS":7, "NEPTUNE":8, "MOON":10, "SUN":11}
try:
    f = open("/home/amigos/ros/src/necst/lib/1st_star_list.txt")
except:
    f = open("/home/necst/ros/src/necst/lib/1st_star_list.txt")
star_list = f.readlines()
planet = ""
target = ""

for i in star_list:
    name, ra, dec = i.split()
    if name.upper() == star.upper():
        target = [float(ra), float(dec)]
    else:
        pass
if len(target) == 0:
    if star.upper() in planet_list:
        planet = star
        pass
    else:
        print('!!Can not find the name of star!!')
        sys.exit()

# observation        
ctrl = ROS_controller.controller()
ctrl.obs_status(active=True, obsmode="Oneshot", obs_script="oneshot.py", obs_file="", target=star)
ctrl.dome_track()
ctrl.move_stop()

now = dt.utcnow()
if planet:
    cplanet = get_body(planet, Time(now))
    cplanet.location = nanten2
    altaz = cplanet.altaz
    azelcoord = SkyCoord(altaz.az.deg, altaz.alt.deg, frame="altaz", unit="deg",obstime=Time(now), location=nanten2)
    radec = azelcoord.fk5

    ctrl.onepoint_move(radec.ra.deg, radec.dec.deg, 'fk5', 0, 0, offcoord="altaz", hosei='hosei_opt.txt', lamda = 0.5)        
    pass
else:
    coord = SkyCoord(target[0],target[1], frame="fk5", unit="deg")
    coord.location = nanten2
    coord.obstime = Time(now)
    altaz = coord.altaz
    ctrl.onepoint_move(target[0], target[1], 'fk5', 0, 0, offcoord="altaz", hosei='hosei_opt.txt', lamda = 0.5)
    pass

ctrl.dome_tracking_check()
ctrl.antenna_tracking_check()

if not filename:
    filename = now.strftime("%Y%m%d_%H%M%S_"+star)
dirname = "/home/amigos/data/experiment/oneshot/" + now.strftime("%Y%m%d") + "/"

ctrl.ccd_oneshot(filename, dirname)
print(dirname, filename)

""" scan test
for j in range(5):
    for i in range(5):
        ctrl.onepoint_move(radec.ra.deg, radec.dec.deg, 'fk5', -2000+i*500, -2000+j*500, offcoord="altaz", hosei='hosei_opt.txt', lamda = 0.5)                
        #ctrl.planet_move(planet, off_x = -6000, off_y=-6500-1000+j*500, hosei = "hosei_opt.txt", lamda = 0.5)
        #ctrl.onepoint_move(target[0], target[1], 'fk5', -5800-3500+500*i, -6300-500+500*j, offcoord="altaz", hosei='hosei_opt.txt', lamda = 0.5)
        #ctrl.onepoint_move(5, 63, 'altaz', -5800-1000+500*i, -6300-1000+500*j, offcoord="altaz", hosei='hosei_opt.txt', lamda = 0.5)        
        ctrl.antenna_tracking_check()
        filename2 = filename + "_x"+str(i) +"_y"+str(j)
        ccd.oneshot(dirname, filename2)
"""

while True:
    if os.path.exists(dirname + filename + '.jpg') == True:
        break
    time.sleep(0.01)
print("###end###")

ctrl.dome_track_end()
ctrl.move_stop()


###show image
try:
    from PIL import Image
    im = Image.open(dirname+filename+".jpg")#from ccd_old.py
    trim_im = im.crop((2080.0, 1360.0, 2720.0, 1840.0))
    trim_im.show()
except Exception as e:
    print(e)


ctrl.obs_status(active=False)
time.sleep(1.5)

print("Finish observation")

