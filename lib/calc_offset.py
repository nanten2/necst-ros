#!/usr/bin/env python3
import astropy.units as u
from astropy.time import Time
from astropy.coordinates import SkyCoord, EarthLocation
from datetime import datetime as dt
import sys
import numpy as np

coord_list = {"icrs":"icrs", "j2000":"fk5", "fk5":"fk5", "b1950":"fk4", "fk4":"fk4", "gal":"galactic", "galactic":"galactic","altaz":"altaz", "horizontal":"altaz","gcrs":"gcrs"}

latitude = -22.96995611
longitude = -67.70308139
height = 4863.85
nanten2 = EarthLocation(lat = latitude*u.deg, lon = longitude*u.deg, height = height*u.m)

def calc_offset(input_x, input_y, coord, input_off_x, input_off_y, offcoord, dcos, timestamp):
    x = input_x*u.deg
    y = input_y*u.deg
    off_x = input_off_x*u.arcsec
    off_y = input_off_y*u.arcsec

    # coordinate check
    try:
        on_frame = coord_list[coord.lower()]
        off_frame = coord_list[offcoord.lower()]
    except:
        print("### !!error coord!! ###")
        sys.exit()
        pass
    
    on_coord = SkyCoord(x, y, frame=on_frame, location=nanten2,obstime=timestamp)
    xsign = 0
    ysign = 0
    
    if off_frame == "altaz":
        xsign = [round((np.sign(i)-1)/2) for i in x]
        ysign = [round((np.sign(i)-1)/2) for i in y]
        on_coord = on_coord.transform_to(off_frame)
        real_az = on_coord.az+360*(xsign*u.deg) + off_x
        real_alt = on_coord.alt+360*(ysign*u.deg) + off_y

        print(real_az)
        xsign = [round((np.sign(i)-1)/2) for i in real_az]
        ysign = [round((np.sign(i)-1)/2) for i in real_alt]
        ret_coord = SkyCoord(real_az, real_alt, frame=off_frame, unit="deg", location=nanten2, obstime=timestamp)
        on_coord = ret_coord.transform_to(on_frame)
        on_x = on_coord.data.lon + 360*(xsign*u.deg)
        on_y = on_coord.data.lat + 360*(ysign*u.deg)
        print(on_x)
    else:
        on_coord = on_coord.transform_to(off_frame)
        param = on_coord.data
        
        if dcos == 1:
            real_lat = param.lat + off_y
            real_lon = param.lon + off_x/np.cos(np.radians(real_lat))
        else:
            real_lat = param.lat + off_y
            real_lon = param.lon + off_x
            pass

        ret_coord = SkyCoord(real_lon, real_lat, frame=off_frame, unit="deg", location=nanten2, obstime=timestamp)
        on_coord = ret_coord.transform_to(on_frame)
        on_x = on_coord.data.lon
        on_y = on_coord.data.lat
        pass

    return [on_x.arcsec, on_y.arcsec, 0, 0]
        
    
