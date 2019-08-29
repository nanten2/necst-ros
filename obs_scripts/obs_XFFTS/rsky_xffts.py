#! /usr/bin/env python3

# Configurations
# ==============

# Info
# ----

name = 'rsky_by_spectrometer'
description = 'Get R-Sky data.'


# Default parameters
# ------------------

integ = 1
memo = ''


# Argument handler
# ================

import argparse

p = argparse.ArgumentParser(description=description)
p.add_argument('--integ', type=float,
               help='Integration time (sec). default=%.1f'%(integ))
p.add_argument('--memo', type=str,
               help='Working memo. This will be include in directroy name.')

args = p.parse_args()

if args.integ is not None: integ = args.integ
if args.memo is not None: memo = args.memo


# Main
# ====

import os
import time
import numpy
import matplotlib.pyplot as plt
import datetime
plt.rcParams['font.size'] = 9

import sys
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")
sys.path.append("/home/amigos/ros/src/nascorx_xffts/")
import ROS_controller
import data_client
d = data_client.data_client()

# Initial configurations
# ----------------------

timestamp = time.strftime('%Y%m%d_%H%M%S')
if memo != '':
    name = name + '_' + memo
else:
    pass
day = time.strftime("%Y%m%d")
savedir = "/home/amigos/data/experiment/rsky/"+day+"/"
if not os.path.exists(savedir):
    os.makedirs(savedir)
else:
    pass


# Data aquisition
# ---------------

con = ROS_controller.controller()
status = con.read_status()
cabin_temp = status.CabinTemp1
if cabin_temp < 10.: # if no data
    cabin_temp = 300
    print("No data from weather")
    print("temporary T=300[K]")
else:
    pass

d1_list = []
d2_list = []

print('Start experimentation')

print('')

print('R')
con.move_hot('in')

status = con.read_status()
hot_status = status.Current_Hot
print('hot_status ### ', hot_status)


while True:
    status = con.read_status()
    hot_status = status.Current_Hot
    print('hot_status ### ',hot_status)
    if not hot_status == 'IN':
        time.sleep(0.5)
        continue
    else:
        break

print('cabin_temp: %.2f'%(cabin_temp))

print('get spectrum...')
"""
d = con.oneshot_achilles(exposure=integ)
#d = {"dfs1":[[100]*16384,0], "dfs2":[[200]*16384,0]}
d1 = d['dfs1'][0]
d2 = d['dfs2'][0]
d1_list.append(d1)
d2_list.append(d2)    
"""
datalist = []
data = d.oneshot(integ,1,time.time()+1)[2][0]
datalist.append(data)

print('SKY')
con.move_hot('out')

status = con.read_status()
hot_status = status.Current_Hot

print('hot_status ### ', hot_status)

while True:
    status = con.read_status()
    hot_status = status.Current_Hot
    if not hot_status == 'OUT':
        time.sleep(0.5)
        continue
    else:
        break


print('get spectrum...')
"""
d = con.oneshot_achilles(exposure=integ)
#d = {"dfs1":[[10]*16384,0], "dfs2":[[20]*16384,0]}
d1 = d['dfs1'][0]
d2 = d['dfs2'][0]
d1_list.append(d1)
d2_list.append(d2)    
"""
data = d.oneshot(integ,1,time.time()+1)[2][0]
datalist.append(data)

con.move_hot('in')

"""
d1_list = numpy.array(d1_list)
d2_list = numpy.array(d2_list)
"""

"""
print('save...')
numpy.save(os.path.join(savedir, '%s_dfs01.npy'%(name)), d1_list)
numpy.save(os.path.join(savedir, '%s_dfs02.npy'%(name)), d2_list)
numpy.savetxt(os.path.join(savedir, '%s_temp.txt'%(name)), [cabin_temp])
"""

def tsys(dhot, dsky, thot):
    #dhot = numpy.array(dhot)
    #dsky = numpy.array(dsky)
    print("#################")
    print(dhot)
    print(dsky)
    y = dhot / dsky
    tsys = thot / (y - 1.)
    return tsys
"""
print(numpy.shape(datalist[0]))
tsys_ = tsys(numpy.array(datalist[0][0])+500000, numpy.array(datalist[1][0]), 300)
import matplotlib.pyplot as plt
plt.plot(tsys_)
plt.show()
"""
datalist[0][0] = numpy.array(datalist[0][0]+100000)
tsys1 = tsys(datalist[0][0], datalist[1][0], cabin_temp)
tsys2 = tsys(datalist[0][1], datalist[1][1], cabin_temp)

#d1_av = numpy.mean(d1_list[:,500:-500], axis=1)
#d2_av = numpy.mean(d2_list[:,500:-500], axis=1)
#tsys1_av = tsys(d1_av[0], d1_av[1], cabin_temp)
#tsys2_av = tsys(d2_av[0], d2_av[1], cabin_temp)
d1_av = 0
d2_av = 1
tsys1_av = 2
tsys2_av = 3

x = numpy.linspace(0, 1000, len(datalist[0][0]))

x = numpy.linspace(0, 1000, len(datalist[0][0]))

#fig = matplotlib.pyplot.figure(figsize=(14, 10))
fig, ax = plt.subplots(2, 2, figsize = (14,10))


#ax11 = fig.add_subplot(211)
#ax12 = fig.add_subplot(212)
#ax21 = fig.add_subplot(221)
#ax22 = fig.add_subplot(222)

#ax11 = ax11.twinx()
#ax12 = ax12.twinx()
#ax21 = ax21.twinx()
#ax22 = ax22.twinx()

ax[0,0].plot(x, datalist[0][0], 'r-')
ax00 = ax[0,0].twinx()
ax00.plot(x, tsys1, ".")
ax[0,1].plot(x, datalist[0][0], 'r-')
ax01 = ax[0,1].twinx()
ax01.plot(x, tsys1, ".")
ax[1,0].plot(x, datalist[0][0], 'r-')
ax10 = ax[1,0].twinx()
ax10.plot(x, tsys1, ".")
ax[1,1].plot(x, datalist[0][0], 'r-')
ax11 = ax[1,1].twinx()
ax11.plot(x, tsys1, ".")



#ax[0,0].plot(x, [6,2,6,4], 'b-')
#ax[0,1].plot(x, [1,1,1,1], 'b-')
#ax[1,0].plot(x, [5,6,6,6], 'b-')
#ax[1,1].plot(x, [2,5,3,8], 'b-')



ax[0,0].set_yscale('log')
ax[0,1].set_yscale('log')
ax[1,0].set_yscale('log')
ax[1,1].set_yscale('log')


ax[0,0].set_xlabel('Freq (MHz)')
ax[0,1].set_xlabel('Freq (MHz)')
ax[1,0].set_xlabel('Freq (MHz)')
ax[1,1].set_xlabel('Freq (MHz)')

ax[0,0].set_ylabel('Power (count)')
ax[0,1].set_ylabel('Power (count)')
ax[1,0].set_ylabel('Power (count)')
ax[1,1].set_ylabel('Power (count)')

ax00.set_ylabel('Tsys [K]')
ax01.set_ylabel('Tsys [K]')
ax10.set_ylabel('Tsys [K]')
ax11.set_ylabel('Tsys [K]')

plt.tight_layout()
plt.show()
