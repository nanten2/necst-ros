#! /usr/bin/env python
# coding:utf-8

# 必要なライブラリのインポート
import ROS_controller
import time
import math
import matplotlib.pylab as plt
from scipy.optimize import curve_fit
import numpy as np
import sys
import signal

con = ROS_controller.controller()
con.get_authority()

# Ctrl+Cで中止するコマンド
def handler(num, flame):
    con.move_stop()
    print("!!Ctrl + C!!")
    print("Stop anntena")
    con.obs_status(active = False)
    sys.exit()

signal.signal(signal.SIGINT, handler)
    
opt = con.read_status()

#HOT点の観測
while not opt.Current_Hot == 'IN':
    opt = con.read_status()
    time.sleep(1)
    con.move_hot('IN')

d_hot_integ1 = []
d_hot_integ2 = []
d_hot_raw = con.oneshot_achilles(repeat = 1, exposure = 1.0, stime = 1.0)
d_hot_dfs1 = d_hot_raw['dfs1']
d_hot_dfs2 = d_hot_raw['dfs2'] 

d_hot_integ1_temp = sum(d_hot_dfs1[0])
d_hot_integ2_temp = sum(d_hot_dfs2[0])

d_hot_integ1.append(d_hot_integ1_temp)
d_hot_integ2.append(d_hot_integ1_temp)

while not opt.Current_Hot == 'OUT':
    opt = con.read_status()
    time.sleep(1)
    con.move_hot('OUT')


opt = con.read_status()


#skydip測定の開始
d_integ1 = []
d_integ2 = []
z = [80, 70, 60, 45, 30, 25, 20]

con.dome_track()
for elevation in z:
    con.onepoint_move(30,elevation)
    time.sleep(5)
    con.dome_tracking_check()
    print("dome track OK")
    con.antenna_tracking_check()
    print("antenna track OK")
    d = con.oneshot_achilles(repeat = 1, exposure = 1.0, stime = 1.0)
    d_dfs1 = d['dfs1']
    d_dfs2 = d['dfs2']
    d_integ_temp1 = sum(d_dfs1[0])
    d_integ_temp2 = sum(d_dfs2[0])
    d_integ1.append(d_integ_temp1)
    d_integ2.append(d_integ_temp2)
    time.sleep(1)    

#解析開始

#log(p_hot-p_sky)のlistの作成
d_hot_sky1 = []
d_hot_sky2 = []

for i in range(len(d_integ1)):
    d_temp = d_hot_integ1[0]*5-d_integ1[i]
    d_temp2 = math.log(d_temp)
    d_hot_sky1 = d_hot_sky1 + [d_temp2]

for i in range(len(d_integ2)):
    d_temp = d_hot_integ2[0]*5-d_integ2[i]
    d_temp2 = math.log(d_temp)
    d_hot_sky2 = d_hot_sky2 + [d_temp2]

#seczの計算

secz = []
for i in range(len(z)):
    secz_temp = (z[i]/180)*math.pi
    secz.append(1/math.cos(secz_temp))

print(d_hot_sky1)
print(d_hot_sky2)
#plot

fig, (axL, axR) = plt.subplots(ncols = 2, figsize = (10,4))

#fitting
fit_array1 = np.polyfit(secz, d_hot_sky1,1)
fit_array2 = np.polyfit(secz, d_hot_sky2,1)


axL.plot(secz, np.poly1d(fit_array1)(secz), label="dfs1")
axL.plot(secz, d_hot_sky1, 'o')
axL.set_title('dfs1')
axL.set_xlabel('secz')
axL.set_ylabel('Power')

axR.plot(secz, np.poly1d(fit_array2)(secz), label="dfs1")
axR.plot(secz, d_hot_sky2, 'o')
axR.set_title('dfs2')
axR.set_xlabel('secz')
axR.set_ylabel('Power')

axR.text(0.1, 0.1, 'tau = {:.4f}'.format(fit_array1[0]), transform = axL.transAxes)
axR.text(1.4, 0.1, 'tau = {:.4f}'.format(fit_array2[0]) , transform = axL.transAxes)

axL.grid()
axR.grid()
plt.tight_layout()
plt.show()

