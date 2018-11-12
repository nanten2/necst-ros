#! /usr/bin/env python
# coding:utf-8

# history
# =======
# ratest edit : kondo



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

args = p.parse_args()

if args.obsfile is not None: obsfile = args.obsfile
if args.tau is not None: tau = args.tau
if args.planet is not None: planet = args.planet
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
#if obs['otadel_off'].lower() == 'y':
    #offset_dcos_off = 1
#else:
    #offset_dcos_off = 0

if obs['coordsys'].lower() == 'j2000' or obs['coordsys'].lower() == 'b1950':
    coord_sys = 'EQUATRIAL'
    #ra = obs['lambda_on']#on点x座標                                           
    #dec = obs['beta_on']#on点y座標     
elif obs['coordsys'].lower() == 'galactic':
    coord_sys = 'GALACTIC'
    #l = obs['lambda_on']#on点x座標                                           
    #b = obs['beta_on']#on点y座標
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
        di = 1887#月の視直径[arcsec]
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

if obs['lo1st_sb_1'] == 'U':#後半に似たのがあるけど気にしない               
   sb1 = 1
else:
    sb1 = -1
if obs['lo1st_sb_2'] == 'U':#後半に似たのがあるけど気にしない               
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
subscan_list = []
lamdel_list = []
betdel_list = []

print('Start experimentation')
print('')

savetime = con.read_status().Time
num = 0
n = int(obs['nSeq'])*4
#line_point = edge*2/obs['grid']
latest_hottime = 0

con.obs_status(active=True, obsmode=obs["obsmode"], obs_script=__file__, obs_file=obsfile, target=planet_name, num_on=obs["N"], num_seq=obs["nSeq"], xgrid=obs["xgrid"], ygrid=obs["ygrid"], exposure_hot=obs["exposure_off"], exposure_off=obs["exposure_off"],exposure_on=obs["exposure"],scan_direction="x")
while num < n: 
    gx = 0#進行方向の識別
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
        subscan= 1
        place = 'right_edge'
    elif num%4 == 2:
        offset_x = 0 +obs["offset_Az"]
        offset_y = -r - edge +obs["offset_El"]
        line_point = obs['N']#int((edge*2)/obs['ygrid'])+1
        gy = 1
        subscan= 2
        place = 'lower_edge'
    elif num%4 == 3:
        offset_x = 0 +obs["offset_Az"]
        offset_y = +r + edge +obs["offset_El"]
        line_point = obs['N']#int((edge*2)/obs['ygrid'])+1
        gy = 1
        subscan= 2
        place = 'upper_edge'
    print('observation : ', place)

    #lp = 0    
    #while lp < line_point:
    #print(place, int(lp)+1)
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
    #dp1 = dp.set_track(obs['lambda_on'], obs['beta_on'], obs['vlsr'], obs['coordsys'], obs['lamdel'], obs['betdel'], offset_dcos, obs['coordsys'], integ_off*2+integ_on, obs['restfreq_1']/1000., obs['restfreq_2']/1000., sb1, sb2, 8038.000000000/1000., 9301.318999999/1000.)
    #lambel_off,betdel_offかも？SYNTHが固定値の場合
    #print(dp1[0])
    status = con.read_status()
    d = con.oneshot_achilles(exposure=integ_off)
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
    vframe_list.append(dp1)#dp1[0])
    vframe2_list.append(dp1)#dp1[0])
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
    _2NDLO_list1.append(dp1)#dp1[3]['sg21']*1000)
    _2NDLO_list2.append(dp1)#dp1[3]['sg22']*1000)
    lamdel_list.append(0)
    betdel_list.append(0)
    subscan_list.append(subscan)
    pass

    #else:
    #dp1 = dp.set_track(obs['lambda_on'], obs['beta_on'], obs['vlsr'], obs['coordsys'], obs['lamdel'], obs['betdel'], offset_dcos, obs['coordsys'], integ_off+integ_on, obs['restfreq_1']/1000., obs['restfreq_2']/1000., sb1, sb2, 8038.000000000/1000., 9301.318999999/1000.)
    #lambel_off,betdel_offかも？SYNTHが固定値の場合
    #pass
    
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
    d = con.oneshot_achilles(exposure=integ_off)
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
    vframe_list.append(dp1)#dp1[0])
    vframe2_list.append(dp1)#dp1[0])
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
    _2NDLO_list1.append(dp1)#dp1[3]['sg21']*1000)
    _2NDLO_list2.append(dp1)#dp1[3]['sg22']*1000)
    lamdel_list.append(0)
    betdel_list.append(0)
    subscan_list.append(subscan)

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
        d = con.oneshot_achilles(exposure=integ_on)
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
        vframe_list.append(dp1)#dp1[0])
        vframe2_list.append(dp1)#dp1[0])
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
        _2NDLO_list1.append(dp1)#dp1[3]['sg21']*1000)    
        _2NDLO_list2.append(dp1)#dp1[3]['sg22']*1000)
        lamdel_list.append(off_x)
        betdel_list.append(off_y)
        subscan_list.append(subscan)

        print('stop')
        con.move_stop()
        lp += 1

    num += 1
    continue

print('R')#最初と最後をhotではさむ
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
d = con.oneshot_achilles(exposure=integ_off)
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
vframe_list.append(dp1)#dp1[0])
vframe2_list.append(dp1)#dp1[0])
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
_2NDLO_list1.append(dp1)#dp1[3]['sg21']*1000)
_2NDLO_list2.append(dp1)#dp1[3]['sg22']*1000)
lamdel_list.append(0)
betdel_list.append(0)
subscan_list.append(subscan)
con.move_hot('out')



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
print(ul1)
cdelt1_1 = (-1)*ul1*0.079370340319607024 #[(km/s)/ch]
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
print(ul2)
cdelt1_2 = (-1)*ul2*0.0830267951512371 #[(km/s)/ch]                           
crpix1_2 = 8191.5 - obs['vlsr']/cdelt1_2 - (500-obs['if3rd_freq_2'])/0.061038881767686015

#planet_number = {1:'mercury', 2:'venus', 4:'mars', 5:'jupiter', 6:'saturn', 7:'uranus', 8:'neptune', 9:'pluto', 10:'moon', 11:'sun'}
#d1list
read1 = {
    "OBJECT" : planet_number[planet],
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
    "SUBSCAN" : subscan_list,
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
    "OBJECT" : planet_number[planet],
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
    "SUBSCAN" : subscan_list,                                                  
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


#print(_2NDLO_list1)
#print(_2NDLO_list2)
f1 = os.path.join(savedir,'n%s_%s_%s_crossedge_%s_pointing.fits'%(timestamp ,obs['molecule_1'] ,obs['transiti_1'].split('=')[1],planet_number[planet]))
f2 = os.path.join(savedir,'n%s_%s_%s_crossedge_%s_pointing.fits'%(timestamp ,obs['molecule_2'] ,obs['transiti_2'].split('=')[1],planet_number[planet]))
#numpy.save(f1+".npy",read1)
#numpy.save(f2+".npy",read2)

import n2fits_write
n2fits_write.write(read1,f1)
n2fits_write.write(read2,f2)

#shutil.copy("/home/amigos/NECST/soft/server/hosei_230.txt", savedir+"/hosei_copy")

con.obs_status(active=False)


import pointing_moon_edge
pointing_moon_edge.analysis(f1) # f2?
