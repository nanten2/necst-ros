import astropy.units as u
from astropy.time import Time
from astropy.coordinates import SkyCoord, EarthLocation
from datetime import datetime as dt
import sys
import numpy as np

coord_list = {"icrs":"icrs", "j2000":"fk5", "fk5":"fk5", "b1950":"fk4", "fk4":"fk4", "gal":"galactic", "galactic":"galactic","altaz":"altaz", "horizontal":"altaz",}

latitude = -22.96995611
longitude = -67.70308139
height = 4863.85
nanten2 = EarthLocation(lat = latitude*u.deg, lon = longitude*u.deg, height = height*u.m)

def calc_offset(input_x, input_y, coord, input_off_x, input_off_y, offcoord, dcos):
    x = input_x*u.deg
    y = input_y*u.deg
    off_x = input_off_x*u.arcsec
    off_x = off_x.to("deg")
    off_x = off_x.value
    off_y = input_off_y*u.arcsec
    off_y = off_y.to("deg")
    off_y = off_y.value
    
    # coordinate check
    try:
        on_frame = coord_list[coord.lower()]
        off_frame = coord_list[offcoord.lower()]
    except:
        print("### !!error coord!! ###")
        pass
    on_coord = SkyCoord(x, y, frame=on_frame, location=nanten2,)
    off_az = 0
    off_el = 0
    
    if off_frame == "altaz":
        print("Offset is horizontal.")
        off_az = input_off_x
        off_el = input_off_y
        pass
    elif off_frame == "galactic":
        on_coord = on_coord.transform_to(off_frame)

        if dcos == 1:
            real_b = on_coord.b.deg + off_y
            real_l = on_coord.l.deg + off_x/np.cos(np.radians(real_b))
        else:
            real_b = on_coord.b.deg + off_y
            real_l = on_coord.l.deg + off_x
            pass

        ret_coord = SkyCoord(real_l, real_b, frame=off_frame, unit="deg")
        on_coord = ret_coord.transform_to(on_frame)
        
    else:
        on_coord = on_coord.transform_to(off_frame)

        if dcos == 1:
            real_dec = on_coord.dec.deg + off_y
            real_ra = on_coord.ra.deg + off_x/np.cos(np.radians(real_dec))
        else:
            real_dec = on_coord.dec.deg + off_y
            real_ra = on_coord.ra.deg + off_x
            pass

        ret_coord = SkyCoord(real_ra, real_dec, frame=off_frame, unit="deg")
        on_coord = ret_coord.transform_to(on_frame)

    if on_frame == "altaz":
        on_x = on_coord.az.arcsec
        on_y = on_coord.alt.arcsec
    elif on_frame == "galactic":
        on_x = on_coord.l.arcsec
        on_y = on_coord.b.arcsec
    else:
        on_x = on_coord.ra.arcsec
        on_y = on_coord.dec.arcsec      
        pass

    return [on_x, on_y, off_az, off_el]
        
    
