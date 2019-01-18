#! /usr/bin/env python
# linearity.py
# -*- coding: utf-8 -*-

'''
=== About this script ====

* [Purpose and Abstract]
To carry out Linearity measurement.
Spectrum is measured for each attenuation level (0-11 [dB]).
Thanks to PATT, changing attenuation level and measurement are all automatically carried out.
This script is written for NECST system with necctrl (172.20.0.11).

* [Devices]
- Programable Attenuator (Model - Driver: 11713C(Agilent), Step Att: 8494H(KEYSIGHT))
- spectrometer (Model: AC240 - dfs01: '172.20.0.41' , 52700
                               dfs02: '172.20.0.43' , 52701)
- ondotori (Model - TR-72W)
- Calculator (necctrl 172.20.0.11 @ Pampa la bola)

* [How to use]
** Execute
On terminal, type like below
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

amigos@necctrl:~/RX$./linearity.py(space)<integtime>

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

** Input
integtime : Integration time of each measurement (Spectrometer).
            Default value is 10 [sec].

** Output
- data (csv file)
- Plot x 2: Raw data x 2 (linear scale and dB scale).
- plot x 11: Difference from references (0-10 [dB]) x 11. (x axis: ch, y axis: power [dB])
- Histogram: for references (0-8 [dB]),
             plot difference between (measured value [dB]) and (attenuation[dB] - reference[dB]). (x axis: difference, y axis: frequency)

This script automatically makes two directories like below whose name are "datetime of experiment + IF 1 or 2".

+++++++ Directory Structure +++++++++++
necctrl:
/home/amigos/RX/
                - linearity_2.py
                - linearity/
                            - yyyymmddHHMM_IF1/
                            - yyyymmddHHMM_IF2/
                                               - raw data (csv file)
                                               - raw data plot x 2 (png file)
                                               - linearity plot x11
                                               - Histogram
+++++++++++++++++++++++++++++++++++++++



* Written by T.Inaba

* [History]
2016/09/29    T.Inaba : modify hot, m4 handler.
2016/08/31    T.Inaba : ver. 1.0
2016/08/28~29 T.Inaba : some miner changes
2016/08/24    T.Inaba : use UT & PATT setting after measurement.
2016/08/17    T.Inaba : miner changes @ histogram
2016/08/10~13 T.Inaba : add statistics
2016/08/09    T.Inaba : beta version
=========================
'''


import time, sys, os
import numpy as np
import matplotlib.pyplot as plt

# default parameter
# ------
integtime = 10

# Argument Handler
# ------
argvs = sys.argv
if len(argvs) != 2:
    integtime = 10
elif len(argvs) == 2:
    integtime = int(argvs[1])

# Conditions
# ------
year = time.strftime("%Y", time.gmtime())
date = time.strftime("%m%d", time.gmtime())
ydatetime = time.strftime("%Y%m%d%H%M", time.gmtime())
datetime = time.strftime("%Y/%m/%d - %H:%M:%S", time.gmtime())
# temp = ond.temp()
# hum = ond.hum()

# Making save directory
# ------
#workdir = "/home/amigos/RX/linearity/"
workdir = "/home/amigos/data/linearity/"
os.mkdir(workdir+ydatetime)

dir_name = workdir+ydatetime+"/"
direname2 = workdir+ydatetime+"_IF2"+"/"

# Devices
# ------
#sys.path.append('/home/amigos/NECRX_system/base_param/')
sys.path.append('/home/amigos/rx/lib/base_param')
import IF
#sys.path.append('/home/amigos/NECRX_system/device_cntrl/')
#import TR72W
#import ac240
#import equipment_nanten
att = IF.prog_att()
#dfs = equipment_nanten.dfs()
#m4 = equipment_nanten.m4()
#hot = equipment_nanten.hot_load()
# ond = TR72W.tr72w()

import ROS_controller
con = ROS_controller.controller()

# Main
# ++++++

# Making save array
# ------
freq = np.linspace(0, 16383, 16384)
sa1 = np.array([freq])   #sa means "spec array"
sa2 = np.array([freq])
sa1_log = np.array([freq])
sa2_log = np.array([freq])

# check M4, HOT and attenuator
# ------
status = con.read_status()
#hot_status = hot.get_status()
#m4_status = m4.get_status()
hot_status = status.Current_Hot
m4_status = status.Current_M4
if hot_status == "OUT":
    con.move_hot('in')
    #hot.move_r()
if m4_status == "OUT":
    con.move_m4('in')
    #m4.m4_in()


while True:#waiting m4&hot IN                               
    status = con.read_status()
    hot_status = status.Current_Hot#v2
    m4_status = status.Current_M4#v2                                                                      
    if not hot_status == 'IN' and not m4_status == 'IN':
        time.sleep(0.5)
        continue
    else:
        break

status = con.read_status()

#hot_status = hot.get_status()
#m4_status = m4.get_status()
hot_status = status.Current_Hot
m4_status = status.Current_M4
att_status = att.get_att()
print(" ")
print("M4 " + m4_status)
print("HOT " + hot_status)
print("IF1: "+str(att_status[0])+" [dB]  IF2: "+str(att_status[1])+" [dB]")
print(" ")

# Start Measurement
# ++++++
for i in range(12):
    att.set_att(i, i)
    time.sleep(1)
    data = con.oneshot_achilles(repeat = 1, exposure=integtime, stime=0)
    #print(sa1.shape)
    #print("###",np.array(data['dfs1'][0]).shape)
    d1 = data['dfs1'][0]
    d2 = data['dfs2'][0]
    np.savez('{}att_{}dB.npz'.format(dir_name,str(i).zfill(2)), dfs1 = d1, dfs2 = d2)
    print(str(i)+"dB finished")

print(" ")
print("Finish measurement")
