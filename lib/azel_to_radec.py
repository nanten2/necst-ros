1#!/usr/bin/env python
import astropy.units as u
from astropy.coordinates import SkyCoord, EarthLocation, AltAz, get_body, FK5
from astropy.time import Time
from datetime import datetime
import kisa_rev
import numpy

#Parameter
latitude = -22.96995611
longitude = -67.70308139
height = 4863.85
nanten2 = EarthLocation(lat = latitude*u.deg, lon = longitude*u.deg, height = height*u.m)

#path
hosei = "/home/amigos/ros/src/necst/lib/hosei_230.txt"

def __e(dt):
    return dt.strftime("%Y-%m-%d %H:%M:%S")

def fk5_from_altaz(az, el, obstime):
    #type check
    #obstime unix time    
    #note
    #pressure needs

    #kisa calc
    delta = []
    for i in range(len(az)):
        delta.append(kisa_rev.apply_kisa_test(az[i]/3600, el[i]/3600, hosei))
    #for broadcasting
    delta = numpy.array(delta)
    delta = delta.transpose((1,0))
    az = numpy.array(az)
    el = numpy.array(el)
    print(az)
    print(el)
    print(delta)
    #az = az + delta[0]/3600#kisa
    #el = el + delta[1]/3600#kisa
    dt = []
    tmp_pressure = 550*100*u.Pa
    for i in obstime:
        dt.append(datetime.utcfromtimestamp(i))
    dd = list(map(__e, dt))
    _time = Time(dd, scale = "utc")
    on_skycoord = SkyCoord(az, el, frame = "altaz", unit = "deg", location = nanten2, obstime = _time, pressure = tmp_pressure)
    t = on_skycoord.transform_to(FK5)
    return t
