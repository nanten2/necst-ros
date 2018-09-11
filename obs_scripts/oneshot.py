#! /usr/bin/env python3

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
        #planet = planet_list[star.upper()]
        planet = star
        pass
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
from astropy.coordinates import get_body,EarthLocation, SkyCoord
from datetime import datetime as dt
from astropy.time import Time
import astropy.units as u
nanten2 = EarthLocation(lat=-22.9699511*u.deg, lon=-67.60308139*u.deg, height=4863.84*u.m)
if planet:
    now = dt.utcnow()
    cplanet = get_body(planet, Time(now))
    cplanet.location = nanten2
    altaz = cplanet.altaz
    azelcoord = SkyCoord(altaz.az.deg, altaz.alt.deg, frame="altaz", unit="deg",obstime=Time(now), location=nanten2)
    radec = azelcoord.fk5

    ctrl.onepoint_move(radec.ra.deg, radec.dec.deg, 'fk5', 0, 0, offcoord="altaz", hosei='hosei_opt.txt', lamda = 0.5)        
    #ctrl.onepoint_move(radec.ra.deg, radec.dec.deg, 'fk5', 0, 0, offcoord="altaz", hosei='hosei_opt.txt', lamda = 0.5)    
    #ctrl.onepoint_move(altaz.az.deg, altaz.alt.deg, 'altaz', -5500, -6600, offcoord="altaz", hosei='hosei_opt.txt', lamda = 0.5)
    #ctrl.planet_move(planet, off_x=-5800, off_y=-6300, hosei = "hosei_opt.txt", lamda = 0.5)
    pass
else:
    now = dt.utcnow()
    coo = SkyCoord(target[0],target[1], frame="fk5", unit="deg")
    coo.location = nanten2
    coo.obstime = Time(now)
    altaz = coo.altaz
    #ctrl.onepoint_move(altaz.az.deg, altaz.alt.deg, 'altaz', -5800, -6300, offcoord="altaz", hosei='hosei_opt.txt', lamda = 0.5)    
    ctrl.onepoint_move(target[0], target[1], 'fk5', 0, 0, offcoord="altaz", hosei='hosei_opt.txt', lamda = 0.5)
    #ctrl.onepoint_move(target[0], target[1], "J2000",lamda = 500)
    pass

b_az = 0
ctrl.dome_tracking_check()#test
ctrl.antenna_tracking_check()#test

if not filename:
    filename = time.strftime("%H%M%S")
#dirname = "/home/amigos/data/experiment/oneshot/" + time.strftime("%Y%m%d")
dirname = time.strftime("%Y%m%d")
if not os.path.exists(dirname):
    os.makedirs(dirname)
ccd.oneshot(dirname, filename)
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

print("###end###")
ctrl.dome_track_end()
ctrl.move_stop()
time.sleep(1)
ctrl.move_stop()

###show image
try:
    from PIL import Image
    time.sleep(2)#wait for picture
    i = Image.open(str(dirname)+'/'+str(filename)+'.bmp')#from ccd_old.py
    i.show()
except Exception as e:
    print(e)
    
print("Finish observation")

