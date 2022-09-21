#!/usr/bin/env python3

# -----------
# 2018/05/23 : kondo
# -----------

from astropy.coordinates import SkyCoord , get_body, EarthLocation
import astropy.units as u
from astropy.time import Time
from datetime import datetime as dt
import sys
import ast
import signal
def handler(signal, frame):
    print("\n")
    print("program is stop.")
    print("\n")
    sys.exit()
signal.signal(signal.SIGINT, handler)

nanten2 = EarthLocation(lat = -22.96995611*u.deg, lon = -67.70308139*u.deg, height = 4863.85*u.m)

planet_list = ['earth', 'sun', 'moon', 'mercury', 'venus', 'earth-moon-barycenter', 'mars', 'jupiter', 'saturn', 'uranus', 'neptune']
planet_dict = {20:'earth', 0:'sun', 10:'moon', 1:'mercury', 2:'venus', 3:'earth-moon-barycenter', 4:'mars', 5:'jupiter', 6:'saturn', 7:'uranus', 8:'neptune'}


def calc(_frame="altaz", now=""):
    if _frame == "altaz":
        x = input("az [deg] : ")
        y = input("el [deg] : ")
    elif _frame in ("fk5", "fk4", "icrs", "gcrs"):
        x = input("ra [deg] : ")
        y = input("dec [deg] : ")
    elif _frame== "galactic":
        x = input("l [deg] : ")
        y = input("b [deg] : ")
    elif _frame == "planet":
        planet = input("planet name : ")
    else:
        print("please, input coord (cf.fk5 or fk4 or galactic or altaz).")
        sys.exit()
    cc = input("change_coord(fk5, fk4, icrs, galactic, altaz) : ")
    if now:
        now = dt.fromtimestamp(float(now))
    else:
        now = dt.utcnow()

    """ calculation"""
    if _frame == "planet":
        if not planet == "all":
            try:
                planet = ast.literal_eval(planet)
            except:
                pass
            if isinstance(planet, int):
                planet = planet_dict[planet]
            coord = get_body(planet,time=Time(now))
            coord.location = nanten2
            coord = coord.gcrs
            new_coord = coord.transform_to(cc)
            lon = new_coord.data.lon.deg
            lat = new_coord.data.lat.deg
        elif planet == "all":
            coord = []
            for i in planet_list:
                _coord = get_body(i,time=Time(now))
                _coord.location = nanten2
                coord.append(_coord.gcrs)
            lon = planet_list
            lat = [coord[i].transform_to(cc) for i in range(len(coord))]
    else:
        if float(x) > 360. or float(x)<-360.:
            _unit="arcsec"
        else:
            _unit="deg"
        coord = SkyCoord(float(x), float(y),frame=_frame,unit=_unit,location=nanten2,obstime=Time(now))
        new_coord = coord.transform_to(cc)
        lon = new_coord.data.lon.deg
        lat = new_coord.data.lat.deg

        ret = coord.transform_to(cc)

    return [_frame, cc, lon, lat]

if __name__=="__main__":
    args = sys.argv
    args.append("")
    args.append("")
    
    if isinstance(args[1], float):
        ret = calc(now=args[1])
    elif not args[1]:
        ret = calc()
    else:
        ret = calc(args[1],args[2])
        pass

    print("\n")
    print("***** result *****")
    print(ret[0] , " --> ", ret[1])
    if not isinstance(ret[2], list): 
        print("longitude : ", round(ret[2], 3), " [deg]")
        print("latitude  : ", round(ret[3], 3), " [deg]")
    else:
        for i in range(len(ret[2])):
            print(ret[2][i], " : (lon, lat) = ", (round(ret[3][i].data.lon.deg,3), round(ret[3][i].data.lat.deg,3)), " [deg]")
            pass
        print("\n")
