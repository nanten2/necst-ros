#! /usr/bin/env python
# coding:utf-8

# Configurations
# ==============
# Info
# ----

name = '_otf_2018'
description = 'Get OTF spectrum'

# Config Parameters
# =================
#path_to_db = "/media/amigos/HD-LCU3/test/write_test/otf20190713.db"
path_to_db = "./hdd/otf20190806_n31.n2df"
#path_to = "/media/amigos/HD-LCU3/test/npy/"

# Default parameters
# ------------------
obsfile = ''
tau = 0.0
c = 299792458

# Argument handler
# ================

import argparse

p = argparse.ArgumentParser(description=description)
p.add_argument('--obsfile', type=str,
               help='absolute path for obsfile')
p.add_argument('--tau', type=float,
               help='tau. default=%.1f'%(tau))

args = p.parse_args()

if args.obsfile is not None: obsfile = args.obsfile
if args.tau is not None: tau = args.tau

# Main
# ====
import os
import shutil
import time
import numpy
from datetime import datetime, timedelta

#from astropy.time import Time # no mojule in python2
import sys
sys.path.append("/home/amigos/necst-obsfiles")
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")
import doppler_nanten
dp = doppler_nanten.doppler_nanten()
import ROS_controller
con = ROS_controller.controller()
con.dome_track()
con.move_stop()
import signal
def handler(num, flame):
    print("!!ctrl+C!!")
    print("STOP MOVING")
    con.move_stop()
    con.dome_stop()
    con.obs_status(active=False)
    time.sleep(1.)
    sys.exit()
signal.signal(signal.SIGINT, handler)


# read obsfile
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
    except:
        try:
            obs[_key] = obs[_value]
        except:
            pass
    if not _value.find('/') == -1 and _value.find('*') == -1:
        _value1,_value2 = _value.split('/',1)
        _value1 = _value1.strip()
        _value2 = _value2.strip()
        if isinstance(_value1,str):
            try:
                _value1 = float(_value1)
            except:
                pass
        else:
            pass
        if isinstance(_value2,str):
            try:
                _value2 = float(_value2)
            except:
                pass
        else:
            pass
        if isinstance(_value1,float) and isinstance(_value2,float):
            obs[_key] = (float(_value1)*100)/(float(_value2)*100)
        elif not isinstance(_value1,float) and not isinstance(_value2,float):
            obs[_key] = (obs[_value1]*100)/(obs[_value2]*100)
        elif isinstance(_value1,float):
            obs[_key] = (float(_value1)*100)/(obs[_value2]*100)
        elif isinstance(_value2,float):
            obs[_key] = (obs[_value1]*100)/(float(_value2)*100)
        else:
            print('Error')
    elif _value.find('/') == -1 and not _value.find('*') == -1:
        _value1,_value2 = _value.split('*',1)
        _value1 = _value1.strip()
        _value2 = _value2.strip()
        if isinstance(_value1,str):
            try:
                _value1 = float(_value1)
            except:
                pass
        if isinstance(_value2,str):
            try:
                _value2 = float(_value2)
            except:
                pass
        if isinstance(_value1,float) and isinstance(_value2,float):
            obs[_key] = (float(_value1)*100)*(float(_value2)*100)/10000
        elif not isinstance(_value1,float) and not isinstance(_value2,float):
            obs[_key] = (obs[_value1]*100)*(obs[_value2]*100)/10000
        elif isinstance(_value1,float):
            obs[_key] = (float(_value1)*100)*(obs[_value2]*100)/10000
        elif isinstance(_value2,float):
            obs[_key] = (obs[_value1]*100)*(float(_value2)*100)/10000
        else:
            print('Error')
    else:
        pass

    continue
# param
# ----------------------
lambda_on = obs['lambda_on']#on
beta_on = obs['beta_on']#on
lambda_off = obs['lambda_off']# off
beta_off = obs['beta_off']# off
coordsys = obs['coordsys'].lower()# coord
dcos = obs['otadel']# dcos
integ_on = obs['exposure']# on iinteg
integ_off = obs['exposure_off']# off integ

#lamdel_on = obs['lamdel']# offset_on
#betdel_on = obs['betdel']# offset_on
lamdel_off = obs['lamdel_off']# offset_off
betdel_off = obs['betdel_off']# offset_off
cosydel = obs['cosydel'].lower()# offset_coord
offset_dcos = obs['otadel_off']# offset_dcos
vlsr = obs["vlsr"]


# Save file
# ----------------------

datahome = '/home/amigos/data/'
timestamp = time.strftime('%Y%m%d%H%M%S')
dirname = 'n%s_%s_%s_otf_%s'%(timestamp ,obs['molecule_1'] ,obs['transiti_1'].split('=')[1],obs['object'])
savedir = os.path.join(datahome, name, dirname)

print('mkdir {savedir}'.format(**locals()))
os.makedirs(savedir)


# Scan Parameters
# --------------- 

sx = obs['lamdel_off']+obs['start_pos_x']#[arcsec]
sy = obs['betdel_off']+obs['start_pos_y']#[arcsec]
direction = int(obs['scan_direction'])

if direction == 0:
    dx = float(obs['otfvel'])*float(obs['exposure'])#[arcsec]
    dy = 0
    gridx = 0
    gridy = obs['grid']#[arcsec]
elif direction ==1:
    dx = 0
    dy = float(obs['otfvel'])*float(obs['exposure'])#[arcsec]
    gridx = obs['grid']#[arcsec]
    gridy = 0
else:
    con.move_stop()
    print('Error:direction')
    sys.exit()
dt = obs['exposure']#float(obs['grid']/obs['otfvel'])

if obs['otadel'].lower() == 'y':
    offset_dcos = 1
    dcos = 1
else:
    offset_dcos = 0
    dcos = 0

if obs['lo1st_sb_1'] == 'U':#後半に似たのがあるけど気にしない
    sb1 = 1
else:
    sb1 = -1
if obs['lo1st_sb_2'] == 'U':#後半に似たのがあるけど気にしない
    sb2 = 1
else:
    sb2 = -1


lamda = c/float(obs['restfreq_1'])#分光計 1
otflen = obs['otflen']/obs['exposure']
scan_point = float(otflen) #scan_point for 1 line
scan_point = int(scan_point)
rampt = dt*obs['lamp_pixels']
print(scan_point)
#if scan_point > int(scan_point):
    #print("!!ERROR scan number!!")

total_count = int(obs['N'])#total scan_line

print('Start experimentation')
print('')

status = con.read_status()
savetime = status.Time


rp = int(obs['nTest'])
rp_num = 0
n = int(total_count)
latest_hottime = 0

# start observation
# ---------------
obsscript = __file__
if obs["scan_direction"] == 0:
    con.obs_status(True, obs["obsmode"], obsscript, obsfile, obs["object"], scan_point, total_count, dx, gridy, integ_off, integ_off, integ_on, "x")
else:
    con.obs_status(True, obs["obsmode"], obsscript, obsfile, obs["object"], scan_point, total_count, gridx, dy, integ_off, integ_off, integ_on, "y")
while rp_num < rp:
    print('repeat : ',rp_num)
    num = 0
    while num < n: 
        print('observation :'+str(num))
        print('tracking start')
        con.move_stop()

        con.onepoint_move(lambda_off, beta_off, coordsys,
                          off_x=lamdel_off, off_y=betdel_off, 
                          offcoord = cosydel,dcos=dcos)

        print("check_track")
        con.antenna_tracking_check()
        con.dome_tracking_check()
        print('tracking OK')

        _now = time.time()
        if _now > latest_hottime+60*obs['load_interval']:
            print('R')
            con.move_hot('in')
            status = con.read_status()
            while status.Current_Hot != "IN":
                print("wait hot_move")
                status = con.read_status()                
                time.sleep(0.5)
            con.obs_status(active=True, current_num=scan_point*num, current_position="HOT")
        
            print('get spectrum...')
            ###con.doppler_calc()
            print(cosydel)

            #con.observation("start", integ_off)
            #time.sleep(integ_off)

            status = con.read_status()
            temp = float(status.CabinTemp1)
            #"""
            #d = con.oneshot_achilles(exposure=integ_off)
            con.xffts_publish_flag(1, path_to_db, str(num), "HOT", 0, 0)
            time.sleep(integ_off)
            con.xffts_publish_flag(0, path_to_db, str(num), "OFF", 0, 0)
            latest_hottime = time.time()
            pass


        else:
            pass
        
        print('OFF')
        con.move_hot('out')
        status = con.read_status()
        while status.Current_Hot != "OUT":
            print("wait hot_move")
            status = con.read_status()                
            time.sleep(0.5)        
        con.obs_status(active=True, current_num=scan_point*num, current_position="OFF")    
        print('get spectrum...')

        status = con.read_status()
        temp = float(status.CabinTemp1)
        #d = con.oneshot_achilles(exposure=integ_off)
        con.xffts_publish_flag(1, path_to_db, str(num), "OFF", 0, 0)
        time.sleep(integ_off)
        con.xffts_publish_flag(0, path_to_db, str(num), "OFF", 0, 0)
        print('move ON')
        con.move_stop()
        ssx = (sx + num*gridx) - float(dx)/float(dt)*rampt-float(dx)/2.#rampの始まり
        ssy = (sy + num*gridy) - float(dy)/float(dt)*rampt-float(dy)/2.#rampの始まり

        print("ramp_start tracking")
        con.onepoint_move(lambda_on, beta_on, coordsys,
                          off_x = ssx, off_y = ssy,
                          offcoord = cosydel,
                          dcos = dcos)
        con.antenna_tracking_check()
        con.dome_tracking_check()
        
        print('reach ramp_start')#rampまで移動

        print(' OTF scan_start!! ')
        print('move ON')
        delay = 3.
        ctime = time.time()
        start_on = 40587 + (ctime+rampt+delay)/24./3600. # mjd
        #print(start_on)
        con.obs_status(active=True, current_num=scan_point*num, current_position="ON")        
        con.otf_scan(lambda_on, beta_on, coordsys, dx, dy, dt, scan_point, rampt, delay=delay, current_time=ctime, off_x = sx + num*gridx, off_y = sy + num*gridy, offcoord = cosydel, dcos=dcos, hosei='hosei_230.txt', lamda=lamda, limit=True)

        print('getting_data...')
        #d = con.oneshot_achilles(repeat = scan_point ,exposure = integ_on ,stime = start_on)
        con.xffts_publish_flag(1, path_to_db, str(num), "ON", 0, 0)
        print("start_on:",start_on)
        while start_on + (obs['otflen']+rampt)/24./3600. > 40587 + time.time()/(24.*3600.):
            #while obs['otflen']/24./3600. > 40587 + time.time()/(24.*3600.):    
            time.sleep(0.001)
        con.xffts_publish_flag(0, path_to_db, str(num), "ON", 0, 0)

        _on = 0
        while _on < scan_point:
            print(_on+1)
            _on += 1

        print('stop')
        con.move_stop()
        num += 1
        continue

    rp_num += 1
    continue

print('R')#最初と最後をhotではさむ
con.move_hot('in')
status = con.read_status()
while status.Current_Hot != "IN":
    print("wait hot_move")
    status = con.read_status()                
    time.sleep(0.5)
con.obs_status(active=True, current_num=scan_point*num+1, current_position="HOT")

#con.observation("start", integ_off)
con.xffts_publish_flag(1, path_to_db, str(num), "HOT", 0, 0)
time.sleep(integ_off)
con.xffts_publish_flag(0, path_to_db, str(num), "HOT", 0, 0)

con.move_hot('out')

print('observation end')
con.move_stop()
con.dome_stop()
