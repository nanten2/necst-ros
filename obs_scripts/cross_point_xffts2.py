#!/usr/bin/env python
# coding:utf-8

###XFFTS version(original is ROS_cross_point.py)

# Configurations
# ==============
# Info
# ----

name = 'radio_pointing_line_9'
description = 'Do radio pointing'

# Default parameters
# ------------------
obsfile = ''
tau = 0.0
#integmin = 8000
#integmax = 9000
#plot_mode = 'plot'
#save_path = '/home/amigos/data/result_png'

# Argument handler
# ================

import argparse

p = argparse.ArgumentParser(description=description)
p.add_argument('--obsfile', type=str,
               help='absolute path for obsfile', required=True)
p.add_argument('--tau', type=float,
               help='tau. default=%.1f'%(tau))
p.add_argument('--integmin', type=int,
               help='integrange_min')
p.add_argument('--integmax', type=int,
               help='integrange_max')
p.add_argument('--plot_mode', type=str,
               help='plot mode : plot/savefig')
p.add_argument('--savepath', type=str,
               help='save path')
args = p.parse_args()

if args.obsfile is not None: obsfile = args.obsfile
if args.tau is not None: tau = args.tau
if args.integmin is not None: integmin = args.integmin
if args.integmax is not None: integmax = args.integmax
if args.plot_mode is not None: plot_mode = args.plot_mode
if args.savepath is not None: savepath = args.savepath

# Main
# ====
import os
import shutil
from datetime import datetime
import sys
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")
sys.path.append("/home/amigos/ros/src/necst/lib")
import time
import signal
import numpy
import doppler_nanten
import log_weather#will be deleted
import logger

#setup logger
#===========
now = datetime.utcnow()
log_path = '/home/amigos/log/{}.txt'.format(now.strftime('%Y%m%d'))
logger = logger.logger(__name__, filename=log_path)
log = logger.setup_logger()
logger.obslog(sys.argv)
start_time = time.time()

dp = doppler_nanten.doppler_nanten()

obs_items = open("/home/amigos/necst-obsfiles/" + obsfile, 'r').read().split('\n')
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
    #except NameError:
    except:
        try:
            obs[_key] = eval(_value, obs)
        except:
            obs[_key] = obs[_value]
            pass
    continue

integ = obs['exposure']
ra = obs['lambda_on']#on点x座標
dec = obs['beta_on']#on点y座標
offx = obs['lambda_off']#off点x座標
offy = obs['beta_off']#off点y座標
xgrid = obs["xgrid"] #offset of pointing(x)
ygrid = obs["ygrid"] #offset of pointing(y)
point_n = obs["N"] #number of line
#point_n = int(point_n / 2) + 1 #number of 1line
if obs['otadel'].lower() == 'y':
    offset_dcos = 1
else:
    offset_dcos = 0
if obs['lo1st_sb_1'] == 'U':#後半に似たのがあるけど気にしない               
   sb1 = 1
else:
    sb1 = -1
if obs['lo1st_sb_2'] == 'U':#後半に似たのがあるけど気にしない               
    sb2 = 1
else:
    sb2 = -1  
if obs['cosydel'].lower() == 'j2000' or obs['cosydel'].lower() == 'b1950':
    cosydel = 'EQUATORIAL'
elif obs['cosydel'].lower() == 'galactic':
    cosydel = 'GALACTIC'
elif obs['cosydel'].lower() == 'horizontal':
    cosydel = 'HORIZONTAL'
else:
    print('cosydel:Error')
    sys.exit()

import ROS_controller
con = ROS_controller.controller()
con.dome_track()
con.xffts_publish_flag()

def handler(num, flame):
    con.move_stop()
    con.dome_stop()
    con.pub_loggerflag("")
    print("!!ctrl + c!!")
    print("Stop antenna")
    time.sleep(1)
    logger.obslog("STOP OBSERVATION", lv=1)
    con.obs_status(active=False)
    sys.exit()

signal.signal(signal.SIGINT, handler)

# Initial configurations
# ----------------------

datahome = '/home/amigos/data/'
timestamp = time.strftime('%Y%m%d%H%M%S')
dirname = 'n%s_%s_%s_cross_%s_pointing'%(timestamp ,obs['molecule_1'] ,obs['transiti_1'].split('=')[1],obs['object'])
savedir = os.path.join("./", name, dirname)
path_to_db = os.path.join(savedir, 'cross.ndf')
print('mkdir {savedir}'.format(**locals()))
os.makedirs(savedir)

#setup weather logger
#====================
print(savedir)
print(os.path.join(savedir, "weather.csv"))
logw = log_weather.Weather_log(os.path.join(savedir, "weather.csv"))
logw.initialize()
def save_weatherlog(scan_num, obs_mode):
    d = con.read_status()
    logw.write(time.time(), d.InTemp, d.OutTemp, d.InHumi, d.OutHumi,
               d.WindDir, d.WindSp, d.Press, d.Rain, d.CabinTemp1, d.CabinTemp2,
               d.DomeTemp1, d.DomeTemp2, d.GenTemp1, d.GenTemp2, scan_num, obs_mode)

print('Start pointing')
print('')
### Logging Start
### =============
con.pub_loggerflag(savedir)

status = con.read_status()
savetime = status.Time
num = 0
n = int(obs['nTest']) * 2
latest_hottime = 0


#for debug
print("xgrid:"+str(xgrid))
print("ygrid:"+str(ygrid))
print("n:"+str(n))
print("point_n:"+str(point_n))


con.obs_status(active=True, obsmode=obs["obsmode"],obs_script=__file__, obs_file=obsfile, target=obs["object"], num_on=obs["N"], num_seq=obs["nTest"], xgrid=obs["xgrid"], ygrid=obs["ygrid"], exposure_hot=obs["exposure"], exposure_off=obs["exposure"], exposure_on=obs["exposure"])
while num < n:
    p_n = 0
    while p_n < point_n:
        save_weatherlog(num, p_n)
        ra = obs['lambda_on']
        dec = obs['beta_on']
        off_x = 0
        off_y = 0
        
        
        print("num "+str(num))
        print("p_n "+str(p_n))
        
        
        if num % 2 == 0:
            off_x = xgrid * (p_n - (int(point_n/2)))
            #lamdel_list.append(xgrid * (p_n - (int(point_n/2))))
            #betdel_list.append(0)
        else:
            off_y = ygrid * (p_n - (int(point_n/2)))
            #lamdel_list.append(0)
            #betdel_list.append(ygrid * (p_n - (int(point_n/2))))
        
        print("ra:"+str(ra))
        print("dec:"+str(dec))
        
        print('observation :'+str(num))
        print('tracking start')
        con.move_stop()
        con.onepoint_move(ra, dec, obs['coordsys'], off_x=off_x+obs["offset_Az"], off_y=off_y+obs["offset_El"], offcoord = cosydel,dcos=1)
        print('moving...')

        con.antenna_tracking_check()
        con.dome_tracking_check()

        #p_n += 1
        
        print('tracking OK')
        _now = time.time()
        if _now > latest_hottime+60*obs['load_interval']:
            print('R')
            #con.move_hot('in')
            con.move_chopper("in")
            time.sleep(3)
            status = con.read_status()
            # while status.Current_Hot != "IN":
            #     print("wait hot_move...")
            #     status = con.read_status()
            #     time.sleep(0.5)                
            con.obs_status(active=True, current_num=num*obs["N"]+p_n, current_position="HOT")        

            status =  con.read_status()
            temp = float(status.CabinTemp1)# + 273.15
            
            print('Temp: %.2f'%(temp))
            
            print('get spectrum...')
            dp1 = dp.set_track(obs['lambda_on'], obs['beta_on'], obs['vlsr'], obs['coordsys'], 0, 0, offset_dcos, obs['coordsys'], integ*2+integ, obs['restfreq_1']/1000., obs['restfreq_2']/1000., sb1, sb2, 8038.000000000/1000., 9301.318999999/1000.)#obs['cosydel']非対応
            con.xffts_publish_flag(scan_num=num + 1, obs_mode="HOT", lamdel=off_x, betdel=off_y)
            time.sleep(integ)
            con.xffts_publish_flag()
        
            print('OFF')
            #con.move_hot('out')
            con.move_chopper("out")
            time.sleep(3)
            status = con.read_status()
            # while status.Current_Hot != "OUT":
            #     print("wait hot_move...")
            #     status = con.read_status()
            #     time.sleep(0.5)            
            con.onepoint_move(offx, offy, obs['coordsys'],off_x=obs["offset_Az"], off_y=obs["offset_El"],dcos=1)
            con.obs_status(active=True, current_num=num*obs["N"]+p_n, current_position="OFF")
        

            con.antenna_tracking_check()
            con.dome_tracking_check()
            print('tracking OK')
            
            
            print('get spectrum...')
            if latest_hottime > _now:
                pass
            else:
                dp1 = dp.set_track(obs['lambda_on'], obs['beta_on'], obs['vlsr'], obs['coordsys'], 0, 0, offset_dcos, obs['coordsys'], integ+integ, obs['restfreq_1']/1000., obs['restfreq_2']/1000., sb1, sb2, 8038.000000000/1000., 9301.318999999/1000.)#obs['cosydel']非対応
                status = con.read_status()
                temp = float(status.CabinTemp1)# + 273.15
                con.xffts_publish_flag(scan_num=num + 1, obs_mode="OFF", lamdel=off_x, betdel=off_y)
                time.sleep(integ)
                con.xffts_publish_flag()
            latest_hottime = time.time()
        
        print('move ON')
        con.move_stop()
        
        con.onepoint_move(ra, dec, obs['coordsys'], off_x = off_x+obs["offset_Az"], off_y = off_y+obs["offset_El"], offcoord = cosydel,dcos=1)
        con.obs_status(active=True, current_num=num*obs["N"]+p_n, current_position="ON")
        
        con.antenna_tracking_check()
        con.dome_tracking_check()
        print('tracking OK')
        
        print('ON')     
        
        print('get spectrum...')
        status = con.read_status()
        temp = float(status.CabinTemp1)# + 273.15
        con.xffts_publish_flag(scan_num=num + 1, obs_mode="ON", lamdel=off_x, betdel=off_y)
        time.sleep(integ)
        con.xffts_publish_flag()
        
        print('stop')
        con.move_stop()
        
        p_n += 1    
    num += 1


# hot->off->on->off->...->on->hot
print('R')
#con.move_hot('in')
con.move_chopper("in")
time.sleep(3)
status = con.read_status()
# while status.Current_Hot != "IN":
#     print("wait hot_move...")
#     status = con.read_status()
#     time.sleep(0.5)    
con.obs_status(active=True, current_num=num*obs["N"]+p_n, current_position="HOT") 

status =  con.read_status()
temp = float(status.CabinTemp1)# + 273.15

print('Temp: %.2f'%(temp))

print('get spectrum...')
dp1 = dp.set_track(obs['lambda_on'], obs['beta_on'], obs['vlsr'], obs['coordsys'], 0, 0, offset_dcos, obs['coordsys'], integ*2+integ, obs['restfreq_1']/1000., obs['restfreq_2']/1000., sb1, sb2, 8038.000000000/1000., 9301.318999999/1000.)#obs['cosydel']非対応
con.xffts_publish_flag(scan_num=num, obs_mode="HOT")
time.sleep(integ)
con.xffts_publish_flag()

#con.move_hot('out')
con.move_chopper("out")
time.sleep(3)

print('observation end')
con.move_stop()
con.dome_stop()

### Logging End
### =============
con.pub_loggerflag("")
logger.obslog('Observation End : observation time : {:.2f\
} [min]'.format((time.time() - start_time)/60), lv=1)
log.info('Observation End : observation time : {:.2f} [mi\
n]'.format((time.time() - start_time)/60))

#shutil.copy("/home/amigos/ros/src/necst/lib/hosei_230.txt", savedir+"/hosei_copy")
#con.obs_status(active=False)

#import pointing_line_xffts
#pointing_line_xffts.analysis(path_to_db, 5000, 25000, 500, 8000, 9000, savepath_filename=os.path.join(savedir, "pointing.png"))
