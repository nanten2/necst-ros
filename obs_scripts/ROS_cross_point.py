#! /usr/bin/env python
# coding:utf-8


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


# Argument handler
# ================

import argparse

p = argparse.ArgumentParser(description=description)
p.add_argument('--obsfile', type=str,
               help='absolute path for obsfile', required=True)
p.add_argument('--tau', type=float,
               help='tau. default=%.1f'%(tau))

args = p.parse_args()

if args.obsfile is not None: obsfile = args.obsfile
if args.tau is not None: tau = args.tau

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

def handler(num, flame):
    con.move_stop()
    con.dome_stop()
    print("!!ctrl + c!!")
    print("Stop antenna")
    con.obs_status(active=False)
    sys.exit()

signal.signal(signal.SIGINT, handler)

# Initial configurations
# ----------------------

datahome = '/home/amigos/data/'
timestamp = time.strftime('%Y%m%d%H%M%S')
dirname = 'n%s_%s_%s_cross_%s_pointing'%(timestamp ,obs['molecule_1'] ,obs['transiti_1'].split('=')[1],obs['object'])
savedir = os.path.join(datahome, name, dirname)

print('mkdir {savedir}'.format(**locals()))
os.makedirs(savedir)

# Data aquisition
# ---------------

d1_list = []
d2_list = []
tdim6_list = []
date_list = []
tsys_list = []
thot_list = []
tcold_list = []
vframe_list = []
vframe2_list = []
lst_list = []
az_list = []
el_list = []
lamdel_list = []
betdel_list = []
tau_list = []
hum_list = []
tamb_list = []
press_list = []
windspee_list = []
winddire_list = []
sobsmode_list = []
mjd_list = []
secofday_list = []
subref_list = []
_2NDLO_list1 = []
_2NDLO_list2 = []

print('Start pointing')
print('')

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
        con.onepoint_move(ra, dec, obs['coordsys'], off_x=off_x, off_y=off_y, offcoord = cosydel)
        print('moving...')

        con.antenna_tracking_check()
        con.dome_tracking_check()

        #p_n += 1
        
        print('tracking OK')
        _now = time.time()
        if _now > latest_hottime+60*obs['load_interval']:
            print('R')
            con.move_hot('in')
            status = con.read_status()
            while not status.Current_Hot != "IN":
                print("wait hot_move...")
                status = con.read_status()
                time.sleep(0.5)                
            con.obs_status(active=True, current_num=num*obs["N"]+p_n, current_position="HOT")        

            status =  con.read_status()
            temp = float(status.CabinTemp1)# + 273.15
            
            print('Temp: %.2f'%(temp))
            
            print('get spectrum...')
            dp1 = dp.set_track(obs['lambda_on'], obs['beta_on'], obs['vlsr'], obs['coordsys'], 0, 0, offset_dcos, obs['coordsys'], integ*2+integ, obs['restfreq_1']/1000., obs['restfreq_2']/1000., sb1, sb2, 8038.000000000/1000., 9301.318999999/1000.)#obs['cosydel']非対応
            d = con.oneshot_achilles(exposure=integ)
            #d = {"dfs1":[[100]*16384, 0], "dfs2":[[200]*16384, 1]}
            d1 = d['dfs1'][0]
            d2 = d['dfs2'][0]
            d1_list.append(d1)
            d2_list.append(d2)
            lamdel_list.append(0)
            betdel_list.append(0)
            tdim6_list.append([16384,1,1])
            tmp_time = status.Time
            tmp2 = datetime.fromtimestamp(tmp_time)
            tmp3 = tmp2.strftime("%Y/%m/%d %H:%M:%S")
            date_list.append(tmp3)            
            thot_list.append(temp)
            vframe_list.append(dp1[0])
            vframe2_list.append(dp1[0])
            lst_list.append(status.LST)
            az_list.append(status.Current_Az)
            el_list.append(status.Current_El)
            tau_list.append(tau)
            hum_list.append(status.OutHumi)
            tamb_list.append(status.OutTemp)
            press_list.append(status.Press)
            windspee_list.append(status.WindSp)
            winddire_list.append(status.WindDir)
            sobsmode_list.append('HOT')
            mjd_list.append(status.MJD)
            secofday_list.append(status.Secofday)
            subref_list.append(status.Current_M2)
            latest_hottime = time.time()
            P_hot = numpy.sum(d1)
            tsys_list.append(0)
            _2NDLO_list1.append(dp1[3]['sg21']*1000)
            _2NDLO_list2.append(dp1[3]['sg22']*1000)
            pass
        
        
        print('OFF')
        con.move_hot('out')
        status = con.read_status()
        while not status.Current_Hot != "OUT":
            print("wait hot_move...")
            status = con.read_status()
            time.sleep(0.5)            
        con.onepoint_move(offx, offy, obs['coordsys'])
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
        d = con.oneshot_achilles(exposure=integ)
        #d = {"dfs1":[[10]*16384, 0], "dfs2":[[20]*16384, 1]}
        d1 = d['dfs1'][0]
        d2 = d['dfs2'][0]
        d1_list.append(d1)
        d2_list.append(d2)
        tdim6_list.append([16384,1,1])
        tmp_time = status.Time
        tmp2 = datetime.fromtimestamp(tmp_time)
        tmp3 = tmp2.strftime("%Y/%m/%d %H:%M:%S")
        date_list.append(tmp3)        
        thot_list.append(temp)
        vframe_list.append(dp1[0])
        vframe2_list.append(dp1[0])
        lst_list.append(status.LST)
        az_list.append(status.Current_Az)
        el_list.append(status.Current_El)
        tau_list.append(tau)
        hum_list.append(status.OutHumi)
        tamb_list.append(status.OutTemp)
        press_list.append(status.Press)
        windspee_list.append(status.WindSp)
        winddire_list.append(status.WindDir)
        sobsmode_list.append('OFF')
        mjd_list.append(status.MJD)
        secofday_list.append(status.Secofday)
        subref_list.append(status.Current_M2)
        P_sky = numpy.sum(d1)
        tsys = temp/(P_hot/P_sky-1)
        tsys_list.append(tsys)
        lamdel_list.append(0)
        betdel_list.append(0)
        _2NDLO_list1.append(dp1[3]['sg21']*1000)
        _2NDLO_list2.append(dp1[3]['sg22']*1000)
        
        print('move ON')
        con.move_stop()
        
        con.onepoint_move(ra, dec, obs['coordsys'], off_x = off_x, off_y = off_y, offcoord = cosydel)
        con.obs_status(active=True, current_num=num*obs["N"]+p_n, current_position="ON")
        

        con.antenna_tracking_check()
        con.dome_tracking_check()
        print('tracking OK')
        
        print('ON')     
        
        print('get spectrum...')
        status = con.read_status()
        temp = float(status.CabinTemp1)# + 273.15
        d = con.oneshot_achilles(exposure=integ)
        #d = {"dfs1":[[10]*16384, 0], "dfs2":[[20]*16384, 1]}
        d1 = d['dfs1'][0]
        d2 = d['dfs2'][0]
        d1_list.append(d1)
        d2_list.append(d2)
        tdim6_list.append([16384,1,1])
        tmp_time = status.Time
        tmp2 = datetime.fromtimestamp(tmp_time)
        tmp3 = tmp2.strftime("%Y/%m/%d %H:%M:%S")
        date_list.append(tmp3)
        thot_list.append(temp)
        vframe_list.append(dp1[0])
        vframe2_list.append(dp1[0])
        lst_list.append(status.LST)
        az_list.append(status.Current_Az)
        el_list.append(status.Current_El)
        tau_list.append(tau)
        hum_list.append(status.OutHumi)
        tamb_list.append(status.OutTemp)
        press_list.append(status.Press)
        windspee_list.append(status.WindSp)
        winddire_list.append(status.WindDir)
        sobsmode_list.append('ON')
        mjd_list.append(status.MJD)
        secofday_list.append(status.Secofday)
        subref_list.append(status.Current_M2)
        tsys_list.append(tsys)
        _2NDLO_list1.append(dp1[3]['sg21']*1000)
        _2NDLO_list2.append(dp1[3]['sg22']*1000)
        if num % 2 == 0:
            #ra += xgrid / 3600. * (p_n - (int(point_n/2)))
            lamdel_list.append(xgrid * (p_n - (int(point_n/2))))
            betdel_list.append(0)
        else:
            #dec += ygrid / 3600. * (p_n - (int(point_n/2)))
            lamdel_list.append(0)
            betdel_list.append(ygrid * (p_n - (int(point_n/2))))    
            
        print('stop')
        con.move_stop()
        
        p_n += 1    
    num += 1


# hot->off->on->off->...->on->hot
print('R')
con.move_hot('in')
status = con.read_status()
while not status.Current_Hot != "IN":
    print("wait hot_move...")
    status = con.read_status()
    time.sleep(0.5)    
con.obs_status(active=True, current_num=num*obs["N"]+p_n, current_position="HOT") 

status =  con.read_status()
temp = float(status.CabinTemp1)# + 273.15

print('Temp: %.2f'%(temp))

print('get spectrum...')
dp1 = dp.set_track(obs['lambda_on'], obs['beta_on'], obs['vlsr'], obs['coordsys'], 0, 0, offset_dcos, obs['coordsys'], integ*2+integ, obs['restfreq_1']/1000., obs['restfreq_2']/1000., sb1, sb2, 8038.000000000/1000., 9301.318999999/1000.)#obs['cosydel']非対応
d = con.oneshot_achilles(exposure=integ)
#d = {"dfs1":[[100]*16384, 0], "dfs2":[[200]*16384, 1]}
d1 = d['dfs1'][0]
d2 = d['dfs2'][0]
d1_list.append(d1)
d2_list.append(d2)
lamdel_list.append(0)
betdel_list.append(0)
tdim6_list.append([16384,1,1])
tmp_time = status.Time
tmp2 = datetime.fromtimestamp(tmp_time)
tmp3 = tmp2.strftime("%Y/%m/%d %H:%M:%S")
date_list.append(tmp3)
thot_list.append(temp)
vframe_list.append(dp1[0])
vframe2_list.append(dp1[0])
lst_list.append(status.LST)
az_list.append(status.Current_Az)
el_list.append(status.Current_El)
tau_list.append(tau)
hum_list.append(status.OutHumi)
tamb_list.append(status.OutTemp)
press_list.append(status.Press)
windspee_list.append(status.WindSp)
winddire_list.append(status.WindDir)
sobsmode_list.append('HOT')
mjd_list.append(status.MJD)
secofday_list.append(status.Secofday)
subref_list.append(status.Current_M2)
latest_hottime = time.time()
P_hot = numpy.sum(d1)
tsys_list.append(0)
_2NDLO_list1.append(dp1[3]['sg21']*1000)
_2NDLO_list2.append(dp1[3]['sg22']*1000)
pass


# ==================================
# save data
# ==================================
if obs['lo1st_sb_1'] == 'U':
    ul = 1
else:
    ul = -1
imagfreq1 = obs['obsfreq_1'] - ul*obs['if1st_freq_1']*2  
lofreq1 = obs['obsfreq_1'] - ul*obs['if1st_freq_1']*1

if obs['lo1st_sb_1'] == 'U':
    ul1_1 = +1
else:
    ul1_1 = -1
if obs['lo2nd_sb_1'] == 'U':
    ul1_2 = +1
else:
    ul1_2 = -1
if obs['lo3rd_sb_1'] == 'U':
    ul1_3 = +1
else:
    ul1_3 = -1
ul1 = ul1_1 * ul1_2 * ul1_3
cdelt1_1 = (-1)*ul1*0.079370340319607024 #[(km/s)/ch]
#dv1 = (300000*cdelt1_1)/obs['restfreq_1']
crpix1_1 = 8191.5 - obs['vlsr']/cdelt1_1 - (500-obs['if3rd_freq_1'])/0.061038881767686015

if obs['lo1st_sb_2'] == 'U':
    ul = 1
else:
    ul = -1
imagfreq2 = obs['obsfreq_2'] - ul*obs['if1st_freq_2']*2
lofreq2 = obs['obsfreq_2'] - ul*obs['if1st_freq_2']*1

if obs['lo1st_sb_2'] == 'U':
    ul2_1 = +1
else:
    ul2_1 = -1
if obs['lo2nd_sb_2'] == 'U':
    ul2_2 = +1
else:
    ul2_2 = -1
if obs['lo3rd_sb_2'] == 'U':
    ul2_3 = +1
else:
    ul2_3 = -1
ul2 = ul2_1 * ul2_2 * ul2_3
cdelt1_2 = (-1)*ul2*0.0830267951512371 #[(km/s)/ch]                                 
#dv2 = (300000*cdelt1_2)/obs['restfreq_2']
crpix1_2 = 8191.5 - obs['vlsr']/cdelt1_2 - (500-obs['if3rd_freq_2'])/0.061038881767686015

#d1list
read1 = {
    "OBJECT" : obs['object'],
    "BANDWID" : 1000000000, #デバイスファイルに追加
    "DATE-OBS" : date_list, 
    "EXPOSURE" : obs['exposure'],
    "TSYS" : tsys_list,
    "DATA" : d1_list,
    "TDIM6" : tdim6_list, #デバイスファイルに追加
    "TUNIT6" : 'counts', #デバイスファイルに追加
    "CTYPE1" : 'km/s', #デバイスファイルに追加
    "CRVAL1" : 0, #デバイスファイルに追加
    "CRPIX1" : crpix1_1, #デバイスファイルに追加
    "CDELT1" : cdelt1_1, #デバイスファイルに追加
    "CTYPE2" : 'deg', #未使用
    "CRVAL2" : 0, #未使用
    "CTYPE3" : 'deg', #未使用
    "CRVAL3" : 0, #未使用
    "T_VLSR" : 0, #未使用
    "OBSERVER" : obs['observer'],
    "SCAN" : 1, #要確認
    "OBSMODE" : obs['obsmode'],
    "MOLECULE" : obs['molecule_1'],
    "TRANSITI" : obs['transiti_1'],
    "TEMPSCAL" : 'TA', #未使用
    "FRONTEND" : 'nagoyaRX', #デバイスファイルに追加
    "BACKEND" : 'nagoyaDFS', #デバイスファイルに追加
    "THOT" : thot_list,
    "TCOLD" : 0, #tcold_list
    "FREQRES" : 0.06103515625, #デバイスファイルに追加[MHz]
    "TIMESYS" : 'UTC', #要確認
    "VELDEF" : 'RADI-LSR',
    "VFRAME" : vframe_list,
    "VFRAME2" : vframe2_list,
    "OBSFREQ" : obs['restfreq_1'], #restfreq_1
    "IMAGFREQ" : imagfreq1, #要計算
    "LST" : lst_list,
    "AZIMUTH" : az_list,
    "ELEVATIO" : el_list,
    "TAU" : tau_list,
    "HUMIDITY" : hum_list,
    "TAMBIENT" : tamb_list,
    "PRESSURE" : press_list,
    "WINDSPEE" : windspee_list,
    "WINDDIRE" : winddire_list,
    "BEAMEFF" : 1, #未使用
    "RESTFREQ" : obs['restfreq_1'],
    "SIG" : 'T', #未使用
    "CAL" : 'F', #未使用
    "SOBSMODE" : sobsmode_list,
    "QUALITY" : 1, #未使用
    "AOSLEN" : 0.04, #未使用
    "LOFREQ" : lofreq1, #要計算
    "SYNTH" : 8038.000000000,#要調査[MHz;IF1]2ndLO
    "FREQSWAM" : 0,#要調査
    "COORDSYS" : obs['coordsys'],
    "COSYDEL" : obs['cosydel'],
    "LAMDEL" : lamdel_list,
    "BETDEL" : betdel_list,
    "OTADEL" : obs['otadel'],
    "OTFVLAM" : 0,
    "OTFVBET" : 0,
    "OTFSCANN" : 0,
    "OTFLEN" : 0,
    "SUBSCAN" : 0, # 要実装
    "MJD" : mjd_list,
    "SECOFDAY" : secofday_list,
    "SIDEBAND" : obs['lo1st_sb_1'],
    "_2NDSB" : obs['lo2nd_sb_1'],
    "_3RDSB" : obs['lo3rd_sb_1'],
    "_2NDLO" : _2NDLO_list1,#ドップラーシフト込み
    "_3RDLO" : obs['lo3rd_freq_1'],
    "SUBREF" : subref_list,
    "LOCKSTAT" : 'F'#未使用
    }

#d2_list
#d1list                                                                        

read2 = {
    "OBJECT" : obs['object'],
    "BANDWID" : 1000000000, #デバイスファイルに追加
    "EXPOSURE" : obs['exposure'],
    "DATE-OBS" : date_list, 
    "TSYS" : tsys_list,
    "DATA" : d2_list,
    "TDIM6" : tdim6_list, #デバイスファイルに追加
    "TUNIT6" : 'counts', #デバイスファイルに追加
    "CTYPE1" : 'km/s', #デバイスファイルに追加 
    "CRVAL1" : 0, #デバイスファイルに追加
    "CRPIX1" : crpix1_2, #デバイスファイルに追加
    "CDELT1" : cdelt1_2, #デバイスファイルに追加
    "CTYPE2" : 'deg', #未使用
    "CRVAL2" : 0, #未使用
    "CTYPE3" : 'deg', #未使用
    "CRVAL3" : 0, #未使用
    "T_VLSR" : 0, #未使用
    "OBSERVER" : obs['observer'],
    "SCAN" : 1, #要確認
    "OBSMODE" : obs['obsmode'],
    "MOLECULE" : obs['molecule_2'],
    "TRANSITI" : obs['transiti_2'],
    "TEMPSCAL" : 'TA', #未使用
    "FRONTEND" : 'nagoyaRX', #デバイスファイルに追加
    "BACKEND" : 'nagoyaDFS', #デバイスファイルに追加                           
    "THOT" : thot_list,
    "TCOLD" : 0, #tcold_list                                                 
    "FREQRES" : 0.06103515625, #デバイスファイルに追加[MHz]                
    "TIMESYS" : 'UTC', #要確認                                                 
    "VELDEF" : 'RADI-LSR',
    "VFRAME" : vframe_list,
    "VFRAME2" : vframe2_list,
    "OBSFREQ" : obs['restfreq_2'],                                
    "IMAGFREQ" : imagfreq2, #要計算                                            
    "LST" : lst_list,
    "AZIMUTH" : az_list,
    "ELEVATIO" : el_list,
    "TAU" : tau_list,
    "HUMIDITY" : hum_list,
    "TAMBIENT" : tamb_list,
    "PRESSURE" : press_list,
    "WINDSPEE" : windspee_list,
    "WINDDIRE" : winddire_list,
    "BEAMEFF" : 1, #未使用                                                     
    "RESTFREQ" : obs['restfreq_2'],
    "SIG" : 'T', #未使用                                                       
    "CAL" : 'F', #未使用                                                       
    "SOBSMODE" : sobsmode_list,
    "QUALITY" : 1, #未使用                                                     
    "AOSLEN" : 0.04, #未使用                                                   
    "LOFREQ" : lofreq2, #要計算                                                
    "SYNTH" : 9301.318999999,#要調査[MHz;IF2]2ndLO                             
    "FREQSWAM" : 0,#要調査                                                     
    "COORDSYS" : obs['coordsys'],
    "COSYDEL" : obs['cosydel'],
    "LAMDEL" : lamdel_list,
    "BETDEL" : betdel_list,
    "OTADEL" : obs['otadel'],
    "OTFVLAM" : 0,
    "OTFVBET" : 0,
    "OTFSCANN" : 0,
    "OTFLEN" : 0,
    "SUBSCAN" : 0, # 要実装                                                    
    "MJD" : mjd_list,
    "SECOFDAY" : secofday_list,
    "SIDEBAND" : obs['lo1st_sb_2'],
    "_2NDSB" : obs['lo2nd_sb_2'],
    "_3RDSB" : obs['lo3rd_sb_2'],
    "_2NDLO" : _2NDLO_list2,#ドップラーシフト込み                              
    "_3RDLO" : obs['lo3rd_freq_2'],
    "SUBREF" : subref_list,
    "LOCKSTAT" : 'F'#未使用                                                    
    }



f1 = os.path.join(savedir,'n%s_%s_%s_cross_%s_pointing.fits'%(timestamp ,obs['molecule_1'] ,obs['transiti_1'].split('=')[1],obs['object']))
f2 = os.path.join(savedir,'n%s_%s_%s_cross_%s_pointing.fits'%(timestamp ,obs['molecule_2'] ,obs['transiti_2'].split('=')[1],obs['object']))

sys.path.append("/home/amigos/ros/src/necst/lib")
import n2fits_write
n2fits_write.write(read1,f1)
n2fits_write.write(read2,f2)


shutil.copy("/home/amigos/ros/src/necst/lib/hosei_230.txt", savedir+"/hosei_copy")
con.obs_status(active=False)
