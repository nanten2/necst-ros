#! /usr/bin/env python
# coding:utf-8


# Configurations
# ==============
# Info
# ----

name = 'ps_edge_pointing'
description = 'Get P/S spectrum'


# Default parameters
# ------------------
obsfile = ''
tau = 0.0
planet = ''
integmin = 5000
integmax = 10000
path_to_db = './edge_20190719_2.db'

# Argument handler
# ================

import argparse

p = argparse.ArgumentParser(description=description)
p.add_argument('--obsfile', type=str,
               help='absolute path for obsfile')
p.add_argument('--tau', type=float,
               help='tau. default=%.1f'%(tau))
p.add_argument('--planet', type=str,
               help='planet_name or planet_number')
p.add_argument('--integmin', type=int,
               help='integrange_min')
p.add_argument('--integmax', type=int,
               help='integrange_max')

args = p.parse_args()

if args.obsfile is not None: obsfile = args.obsfile
if args.tau is not None: tau = args.tau
if args.planet is not None: planet = args.planet
if args.integmin is not None: integmin = args.integmin
if args.integmax is not None: integmax = args.integmax
print(planet)

# Main
# ====
import os
import shutil
import time
import math
import numpy
from datetime import datetime
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
import doppler_nanten
dp = doppler_nanten.doppler_nanten()
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")
import ROS_controller

con = ROS_controller.controller()
con.dome_track()
import signal
def handler(num, flame):
    print("!!ctrl+C!!")
    print("STOP MOVING")
    con.dome_stop()
    con.move_stop()
    con.obs_status(active=False)    
    sys.exit()
signal.signal(signal.SIGINT, handler)

obsdir = '/home/amigos/necst-obsfiles/'
obs_items = open(obsdir+obsfile, 'r').read().split('\n')
obs = {}
for _item in obs_items:
    print(_item)
    if _item.startswith('script;'): break
    _item = _item.split('#')[0]
    _key, _value = _item.split('=', 1)
    _key = _key.strip()
    _value = _value.strip()
    try:
        obs[_key] = eval(_value)
    except NameError:
        obs[_key] = obs[_value]
        pass
    continue

integ_on = obs['exposure']
integ_off = obs['exposure_off']
edge = math.fabs(obs['edge'])
if obs['otadel'].lower() == 'y':
    offset_dcos = 1
else:
    offset_dcos = 0

if obs['coordsys'].lower() == 'j2000' or obs['coordsys'].lower() == 'b1950':
    coord_sys = 'EQUATRIAL'

elif obs['coordsys'].lower() == 'galactic':
    coord_sys = 'GALACTIC'

elif obs['coordsys'].lower() == 'planet':
    coord_sys = 'PLANET'
    try:
        planet = int(planet)
        print(planet)
    except:
        print(planet)
        pass
    if isinstance(planet,str):
        planet_name = planet.lower()
        planet_number = {'mercury':1, 'venus':2, 'mars':4, 'jupiter':5, 'saturn':6, 'uranus': 7, 'neptune':8, 'pluto':9, 'moon':10, 'sun':11}
        planet = planet_number[planet_name]
    elif isinstance(planet,int):
        planet = int(planet)
    else:
        print('planet_name Error')
        quit()
    if planet == 10:
        di = 1887#?????[arcsec]
        r = di/2
    elif planet == 11:
        di = 1920#sun_r[arcsec]
        r =di/2
    else:
        print('please : sun(10) or moon(11)') 
        sys.exit()
else:
    print('Error:coordsys')
    con.move_stop()
    sys.exit()

if obs['cosydel'].lower() == 'j2000' or obs['cosydel'].lower() == 'b1950':
    cosydel = 'EQUATORIAL'
elif obs['cosydel'].lower() == 'galactic':
    cosydel = 'GALACTIC'
elif obs['cosydel'].lower() == 'horizontal':
    cosydel = 'HORIZONTAL'
else:
    print('cosydel:Error')
    sys.exit()

if obs['lo1st_sb_1'] == 'U':#????????????????               
   sb1 = 1
else:
    sb1 = -1
if obs['lo1st_sb_2'] == 'U':#????????????????               
    sb2 = 1
else:
    sb2 = -1  


# Initial configurations
# ----------------------

datahome = '/home/amigos/data'
timestamp = time.strftime('%Y%m%d%H%M%S')
planet_number = {1:'mercury', 2:'venus', 4:'mars', 5:'jupiter', 6:'saturn', 7:'uranus', 8:'neptune', 9:'pluto', 10:'moon', 11:'sun'}
dirname = 'n%s_%s_%s_crossedge_%s_pointing'%(timestamp ,obs['molecule_1'] ,obs['transiti_1'].split('=')[1],planet_number[planet])
savedir = os.path.join(datahome, name, dirname)

print('mkdir {savedir}'.format(**locals()))
os.makedirs(savedir)

print('Start experimentation')
print('')

savetime = con.read_status().Time
num = 0
n = int(obs['nSeq'])*4
latest_hottime = 0

con.obs_status(active=True, obsmode=obs["obsmode"], obs_script=__file__, obs_file=obsfile, target=planet_name, num_on=obs["N"], num_seq=obs["nSeq"], xgrid=obs["xgrid"], ygrid=obs["ygrid"], exposure_hot=obs["exposure_off"], exposure_off=obs["exposure_off"],exposure_on=obs["exposure"],scan_direction="x")
while num < n: 
    gx = 0#???????
    gy = 0
    #print('observation :'+str(num))
    if num%4 == 0:
        offset_x = -r - edge +obs["offset_Az"]
        offset_y = 0 +obs["offset_El"]
        line_point = obs['N']
        gx = 1
        subscan = 1
        place = 'left_edge'
    elif num%4 == 1:
        offset_x = +r + edge +obs["offset_Az"]
        offset_y = 0 +obs["offset_El"]
        line_point = obs['N']#int((edge*2)/obs['xgrid'])+1
        gx = 1
        subscan= 2
        place = 'right_edge'
    elif num%4 == 2:
        offset_x = 0 +obs["offset_Az"]
        offset_y = -r - edge +obs["offset_El"]
        line_point = obs['N']#int((edge*2)/obs['ygrid'])+1
        gy = 1
        subscan= 3
        place = 'lower_edge'
    elif num%4 == 3:
        offset_x = 0 +obs["offset_Az"]
        offset_y = +r + edge +obs["offset_El"]
        line_point = obs['N']#int((edge*2)/obs['ygrid'])+1
        gy = 1
        subscan= 4
        place = 'upper_edge'
    print('observation : ', place)

    print('tracking start')
    con.move_stop()
    
    if coord_sys == 'PLANET':
        print(planet)
        con.planet_move(planet, off_x=offset_x, off_y=offset_y, dcos = offset_dcos, offcoord = cosydel)
        print('off_x : ',offset_x)
        print('off_y : ',offset_y)
        print('moving...')
        con.obs_status(active=True, current_num=num*obs["N"], current_position="HOT")
    else:
        pass

    con.antenna_tracking_check()
    con.dome_tracking_check()
    print('tracking OK')
    
    _now = time.time()
    #if _now > latest_hottime+60*obs['load_interval']:
    print('R')
    con.move_hot('in')
    status = con.read_status()
    while status.Current_Hot != "IN":
        print("wait hot_move...")
        status = con.read_status()        
        time.sleep(0.5)
            
    temp = float(con.read_status().CabinTemp1)# + 273.15
        
    print('Temp: %.2f'%(temp))
    print('get spectrum...')
    dp1 = 0
    status = con.read_status()
    con.xffts_publish_flag(1, path_to_db, str(subscan), "HOT", 0, 0)
    time.sleep(integ_off)
    con.xffts_publish_flag(0, path_to_db, str(subscan), "HOT", 0, 0)
    pass

    print('OFF')
    con.move_hot('out')
    status = con.read_status()
    while status.Current_Hot != "OUT":
        print("wait hot_move...")
        status = con.read_status()        
        time.sleep(0.5)    
    print('get spectrum...')
    con.obs_status(active=True, current_num=num*obs["N"], current_position="OFF")
    status = con.read_status()
    temp = float(status.CabinTemp1)# + 273.15
    con.xffts_publish_flag(1, path_to_db, str(subscan), "OFF", 0, 0)
    time.sleep(integ_off)
    con.xffts_publish_flag(0, path_to_db, str(subscan), "OFF", 0, 0)

    lp = 0
    while lp < line_point:
        print(place, int(lp)+1)
        print('move ON')
        con.move_stop()

        if coord_sys == 'EQUATRIAL':
            pass
        elif coord_sys == 'GALACTIC':
            pass
        elif coord_sys == 'PLANET':
            if num%4 == 1 or num%4 == 3 :
                print('right or upper')                
                off_x = offset_x + (-2*edge+obs['xgrid']*lp)*gx
                off_y = offset_y + (-2*edge+obs['ygrid']*lp)*gy
            else:
                print('left or lower')
                off_x = offset_x + (obs['xgrid']*lp)*gx
                off_y = offset_y + (obs['ygrid']*lp)*gy
            con.planet_move(planet, off_x = off_x,off_y = off_y, 
                            offcoord = cosydel,dcos = offset_dcos)
            print('off_x : ', off_x)
            print('off_y : ', off_y)

        con.obs_status(active=True, current_num=num*obs["N"]+lp, current_position="ON")
        con.antenna_tracking_check()
        con.dome_tracking_check()
        print('tracking OK')
        
        print('ON')     
        
        print('get spectrum...')
        status = con.read_status()
        temp = float(status.CabinTemp1)# + 273.15
        con.xffts_publish_flag(1, path_to_db, str(subscan), "ON", off_x, off_y)
        time.sleep(integ_off)
        con.xffts_publish_flag(0, path_to_db, str(subscan), "ON", off_x, off_y)
    
        print('stop')
        con.move_stop()
        lp += 1

    num += 1
    continue

print('R')#??????hot????
con.move_hot('in')
status = con.read_status()
while status.Current_Hot != "IN":
    print("wait hot_move...")
    status = con.read_status()        
    time.sleep(0.5)
con.obs_status(active=True, current_num=num*obs["N"], current_position="HOT")
status = con.read_status()
temp = float(status.CabinTemp1)# + 273.15
        
print('Temp: %.2f'%(temp))
print('get spectrum...')
con.xffts_publish_flag(1, path_to_db, str(subscan), "HOT", 0, 0)
time.sleep(integ_off)
con.xffts_publish_flag(0, path_to_db, str(subscan), "HOT", 0, 0)

con.move_hot('out')
print('observation end')
con.move_stop()
con.dome_stop()

import pointing_edge_xffts
#t = time.time()
pointing_edge_xffts.analysis('/home/amigos/ros/src/necst/scripts/record/edge_20190719_2.db', integ_mi=integmin, integ_ma=integmax) 
#print(time.time() - t)
