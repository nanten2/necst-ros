#! /usr/bin/env python
# coding:utf-8


# Configurations
# ==============
# Info
# ----

name = 'focus_planet'
description = 'Get P/S spectrum'


# Default parameters
# ------------------
obsfile = ''
tau = 0.0
planet = ""


# Argument handler
# ================

import argparse

p = argparse.ArgumentParser(description=description)
p.add_argument('--obsfile', type=str,
               help='absolute path for obsfile')
p.add_argument('--tau', type=float,
               help='tau. default=%.1f'%(tau))
p.add_argument('--planet', type=str,
               help='planet_name')

args = p.parse_args()

if args.obsfile is not None: obsfile = args.obsfile
if args.tau is not None: tau = args.tau
if args.planet is not None: planet = args.planet

# Main
# ====

# Preparetion                                                                                  
# ------------------                                                                                 

import os
import time
import numpy
from datetime import datetime
import sys
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
    try:
        status = con.read_status()
        print(start_m2.Current_M2 - status.Current_M2)
        dist = round(start_m2.Current_M2 - status.Current_M2,3)
        print(dist)
        con.move_m2(dist*1000)
        print("*** m2 move : ", dist, " [um] ***")
        print("m2 move start position")
    except:
        print("m2 don't move...")
        pass
    con.obs_status(active=False)
    time.sleep(3.)
    sys.exit()
signal.signal(signal.SIGINT, handler)

###obsdir = '/home/amigos/NECST/script/obslist/ps/'
obsdir = "/home/amigos/necst-obsfiles/"
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

# Use obs_file
# ------------------

offset_Az = obs['offset_Az']
offset_El = obs['offset_El']
vlsr = obs['vlsr']

lambda_on = obs['lambda_on']# on
beta_on = obs['beta_on']# on
lambda_off = obs['lambda_off']# off
beta_off = obs['beta_off']# off
coordsys = obs['coordsys'].lower()# coord
dcos = obs['otadel']# dcos
integ_on = obs['exposure']# on iinteg
integ_off = obs['exposure_off']# off integ

lamdel_on = obs['lamdel']# offset_on
betdel_on = obs['betdel']# offset_on
lamdel_off = obs['lamdel_off']# offset_off
betdel_off = obs['betdel_off']# offset_off
cosydel = obs['cosydel'].lower()# offset_coord
offset_dcos = obs['otadel_off']# offset_dcos

if dcos.lower() == 'y':
    dcos = 1
else:
    dcos = 0
if offset_dcos.lower() == 'y':
    off_dcos = 1
else:
    off_dcos = 0

if obs['lo1st_sb_1'] == 'U':#後半に似たのがあるけど気にしない()               
   sb1 = 1
else:
    sb1 = -1
if obs['lo1st_sb_2'] == 'U':#後半に似たのがあるけど気にしない()               
    sb2 = 1
else:
    sb2 = -1  


datahome = '/home/amigos/data/test/'
timestamp = time.strftime('%Y%m%d_%H%M%S')
dirname = timestamp
savedir = os.path.join(datahome, name, dirname)

print('mkdir {savedir}'.format(**locals()))
os.makedirs(savedir)


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

print('Start experimentation')
print('')

savetime = con.read_status().Time
num = 0
n = int(obs['nSeq'])
latest_hottime = 0
start_m2 = con.read_status()

print("moving m2...")
dist = -400*int(n/2)
con.move_m2(dist)
print("*** m2 move : ",  dist, " [ um ] ***")
now = con.read_status()
while abs(now.Current_M2 - (start_m2.Current_M2 + dist/1000)) > 0.03:
    print(now.Current_M2,(start_m2.Current_M2 + dist/1000))
    print("moving m2...")
    time.sleep(0.1)
    now = con.read_status()        

con.obs_status(active=True, obsmode=obs["obsmode"], obs_script=__file__, obs_file=obsfile, target=obs["object"], num_on=obs["nON"], num_seq=obs["nSeq"], exposure_hot=obs["exposure_off"], exposure_off=obs["exposure_off"], exposure_on=obs["exposure"])
while num < n:
    print("moving m2...")
    if num == 0:
        dist = 0
    else:
        dist = 400
    con.move_m2(dist)
    print("*** m2 move : ",  dist, " [ um ] ***")
    now = con.read_status()
    while abs(now.Current_M2 - (start_m2.Current_M2 +(-400*int(n/2))/1000 + dist*num/1000)) > 0.03:
        print("moving m2...")
        time.sleep(0.1)
        now = con.read_status()        
    
    print('observation :'+str(num+1) + "\n")
        
    con.planet_move(planet, off_x=1200,off_y=1200,dcos=1)

    con.antenna_tracking_check()
    con.dome_tracking_check()
    print('tracking OK'+ "\n")

    _now = time.time()
    if _now > latest_hottime+60*obs['load_interval']:
        print('R'+ "\n")
        
        con.move_hot('in')
        status = con.read_status()
        while status.Current_Hot != "IN":
            print("wait hot_move")
            time.sleep(0.5)
            status = con.read_status()
        con.obs_status(active=True, current_num=num, current_position="HOT")               

        print('get spectrum...')
        ###con.doppler_calc()
        #dp1 = dp.set_track(lambda_on, beta_on, vlsr, coordsys, lamdel_on, betdel_on, dcos, cosydel, 
                           #integ_off*2, obs['restfreq_1']/1000., obs['restfreq_2']/1000., 
                           #sb1, sb2, 8038.000000000/1000., 9301.318999999/1000.)
        dp1 = [0,0,0,{"sg21":0,"sg22":0}] 
        time.sleep(integ_off)

        status = con.read_status()
        temp = float(status.CabinTemp1)# + 273.15
        d = con.oneshot_achilles(exposure=integ_off)
        #d = {'dfs1': [[100]*16384,1], 'dfs2': [[10]*16384,11]}
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
        sobsmode_list.append('HOT')
        mjd_list.append(status.MJD)
        secofday_list.append(status.Secofday)
        subref_list.append(status.Current_M2)
        latest_hottime = time.time()
        P_hot = numpy.sum(d1)
        tsys_list.append(0)
        _2NDLO_list1.append(dp1[3]['sg21']*1000)
        _2NDLO_list2.append(dp1[3]['sg22']*1000)
        #print("sg21 : ", dp1[3]['sg21']*1000)
        #print("sg22 : ", dp1[3]['sg22']*1000, "\n")

        pass


    else:
        #dp1 = dp.set_track(lambda_on, beta_on, vlsr, coordsys, lamdel_on, betdel_on, dcos, cosydel,
                           #integ_off, obs['restfreq_1']/1000., obs['restfreq_2']/1000., 
                           #sb1, sb2, 8038.000000000/1000., 9301.318999999/1000.)
        dp1 = [0,0,0,{"sg21":0,"sg22":0}]         
        pass
    print('OFF'+ "\n")
    con.move_hot('out')
    status = con.read_status()
    while status.Current_Hot != "OUT":
        print("wait hot_move")
        time.sleep(0.5)
        status = con.read_status()    
    con.obs_status(active=True, current_num=num, current_position="OFF")
    print('get spectrum...')
    #con.observation("start", integ_off)# getting one_shot_data
    time.sleep(integ_off)

    status = con.read_status()
    temp = float(status.CabinTemp1)# + 273.15
    d = con.oneshot_achilles(exposure=integ_off)
    #d = {'dfs1': [[1]*16384,1], 'dfs2': [[10]*16384,11]}
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
    _2NDLO_list1.append(dp1[3]['sg21']*1000)
    _2NDLO_list2.append(dp1[3]['sg22']*1000)
    #print("sg21 : ", dp1[3]['sg21']*1000)
    #print("sg22 : ", dp1[3]['sg22']*1000, "\n")

    print('move ON'+ "\n")

    con.planet_move(planet, dcos=1)

    con.obs_status(active=True, current_num=num, current_position="ON")    
    con.antenna_tracking_check()
    con.dome_tracking_check()
    print('tracking OK'+"\n")

    print('get spectrum...')

    status = con.read_status()
    temp = float(status.CabinTemp1)# + 273.15
    d = con.oneshot_achilles(exposure=integ_on)
    #d = {'dfs1': [[1]*16384,1], 'dfs2': [[10]*16384,11]}
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
    #print("sg21 : ", dp1[3]['sg21']*1000)
    #print("sg22 : ", dp1[3]['sg22']*1000,"\n")
    print('stop'+"\n")
        
    num += 1
    continue

print("moving m2...")
dist = -400*int(n/2)
con.move_m2(dist)
print("*** m2 move : ", dist,  " [um] ***")
now = con.read_status()
if now.Current_M2 == start_m2.Current_M2:
    print("moving m2...")
    time.sleep(0.1)
    now = con.read_status()        

print('R'+"\n")#最初と最後をhotではさむ
con.move_hot('in')
status = con.read_status()
while status.Current_Hot != "IN":
    print("wait hot_move")
    time.sleep(0.5)
    status = con.read_status()
con.obs_status(active=True, current_num=num, current_position="HOT")

status = con.read_status()
temp = float(status.CabinTemp1)# + 273.15
print('Temp: %.2f'%(temp))
print('get spectrum...')
d = con.oneshot_achilles(exposure=integ_off)
#d = {'dfs1': [[100]*16384,1], 'dfs2': [[10]*16384,11]}
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
sobsmode_list.append('HOT')
mjd_list.append(status.MJD)
secofday_list.append(status.Secofday)
subref_list.append(status.Current_M2)
P_hot = numpy.sum(d1)
tsys_list.append(0)
_2NDLO_list1.append(dp1[3]['sg21']*1000)
_2NDLO_list2.append(dp1[3]['sg22']*1000)
#print("sg21 : ", dp1[3]['sg21']*1000)
#print("sg22 : ", dp1[3]['sg22']*1000, "\n")

con.move_hot('out')
print('observation end'+"\n")
con.move_stop()
con.dome_stop()


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
#print(ul1)
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
#print(ul2)
cdelt1_2 = (-1)*ul2*0.0830267951512371 #[(km/s)/ch]                           
#dv2 = (300000*cdelt2)/obs['restfreq_2']
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
    "LAMDEL" : obs['lamdel'],
    "BETDEL" : obs['betdel'],
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
    "LAMDEL" : obs['lamdel'],
    "BETDEL" : obs['betdel'],
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



f1 = os.path.join(savedir,'n2ps_%s_IF1.fits'%(timestamp))
f2 = os.path.join(savedir,'n2ps_%s_IF2.fits'%(timestamp))
#numpy.save(f1+".npy",read1)
#numpy.save(f2+".npy",read2)


sys.path.append("/home/amigos/ros/src/necst/lib")
import n2fits_write
n2fits_write.write(read1,f1)
n2fits_write.write(read2,f2)

timestamp = time.strftime('%Y%m%d_%H%M%S')
dirname = timestamp
con.obs_status(active=False)
