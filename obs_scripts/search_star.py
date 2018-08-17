#!/usr/bin/env python3

from astropy.coordinates import SkyCoord , get_body, EarthLocation, AltAz, FK5
import astropy.units as u
from astropy.time import Time
from datetime import datetime as dt
from datetime import timedelta
import sys
import time
nanten2 = EarthLocation(lat = -22.96995611*u.deg, lon = -67.70308139*u.deg, height = 4863.85*u.m)

''' 1st_star list '''
f = open("/home/amigos/ros/src/necst/lib/1st_star_list.txt","r")
ff = f.readlines()
f.close()
''' planet list '''
planet_list = ['sun', 'moon', 'mercury', 'venus', 'mars', 'jupiter', 'saturn', 'uranus', 'neptune']

star = []
ra = []
dec = []
for i in range(len(ff)):
    k = ff[i].split()
    star.append(k[0])
    ra.append(float(k[1]))
    dec.append(float(k[2]))

ctime = dt.utcnow()
coord = SkyCoord(ra, dec, unit="deg", frame="fk5", obstime=Time(ctime), location=nanten2)
planet_altaz = []
for i in planet_list:
    planet_coord = get_body(i,time=Time(ctime))
    planet_coord.location = nanten2
    planet_altaz.append(planet_coord.altaz)
            

altaz = coord.transform_to(AltAz)
print("*** rising star list [deg] ***")
for i in range(len(altaz)):
    if 20 < altaz[i].alt.deg < 80:
        print(star[i]," "*(11-len(star[i])) , ": (az, el) = (", round(altaz[i].az.deg, 4), ", ", round(altaz[i].alt.deg, 4), ")")
print("==============================")
for i in range(len(planet_altaz)):
    if 20 < planet_altaz[i].alt.deg < 80:
        print(planet_list[i]," "*(8-len(planet_list[i])) , ": (az, el) = (", round(planet_altaz[i].az.deg, 4), ", ", round(planet_altaz[i].alt.deg, 4), ")")
print("******************************")
