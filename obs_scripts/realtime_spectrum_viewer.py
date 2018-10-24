#! /usr/bin/env python3

# Configurations
# ==============

# Info
# ----

name = 'spectrometer'
description = 'Get row data.'


# Default parameters
# ------------------

integ = 1
memo = ''
mode = "sky"

# Argument handler
# ================

import argparse

p = argparse.ArgumentParser(description=description)
p.add_argument('--integ', type=float,
               help='Integration time (sec). default=%.1f'%(integ))
p.add_argument('--memo', type=str,
               help='Working memo. This will be include in directroy name.')
p.add_argument('--mode', type=str,
               help='getting data mode (r/sky). default=sky')

args = p.parse_args()

if args.integ is not None: integ = args.integ
if args.memo is not None: memo = args.memo
if args.mode is not None: mode = args.mode

# Main
# ====

import os
import time
import numpy
import matplotlib.pyplot
import datetime
matplotlib.pyplot.rcParams['font.size'] = 9

import sys
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")
import ROS_controller


# Initial configurations
# ----------------------

if memo != '':
    name = name+"_"+memo
else:
    pass

day = time.strftime("%Y%m%d")    
savedir = "/home/amigos/data/experiment/spectrometer/"+day + "/"
if not os.path.exists(savedir):
    os.makedirs(savedir)
else:
    pass


# Data aquisition
# ---------------

con = ROS_controller.controller()
status = con.read_status()
cabin_temp = status.CabinTemp1

d1_list = []
d2_list = []

print('Start experimentation')

print('')

if mode == "sky":
    print("sky")
    con.move_hot("out")
else:
    print('R')
    con.move_hot('in')

time.sleep(1)

print('cabin_temp: %.2f'%(cabin_temp))

print('get spectrum...')

"""
d1_list = numpy.array(d1_list)
d2_list = numpy.array(d2_list)

print('save...')
numpy.save(os.path.join(savedir, '%s_dfs01.npy'%(name)), d1_list)
numpy.save(os.path.join(savedir, '%s_dfs02.npy'%(name)), d2_list)
numpy.savetxt(os.path.join(savedir, '%s_temp.txt'%(name)), [cabin_temp])
"""

def tsys(dhot, dsky, thot):
    dhot = numpy.array(dhot)
    dsky = numpy.array(dsky)
    
    y = dhot / dsky
    tsys = thot / (y - 1.)
    return tsys


x = numpy.arange(1, 16384+1)

fig = matplotlib.pyplot.figure(figsize=(14, 5))
ax11 = fig.add_subplot(121)
ax21 = fig.add_subplot(122)
ax12 = ax11.twinx()
ax22 = ax21.twinx()

ax11.set_xlabel('Freq (ch)')
ax21.set_xlabel('Freq (ch)')
ax11.set_ylabel('Power (count [log])')
ax21.set_ylabel('Power (count [log])')
ax11.set_title('dfs01')
ax21.set_title('dfs02')
ax11.set_yscale('log')
ax21.set_yscale('log')

interbal = 0.1

d = con.oneshot_achilles(exposure=integ)
d1 = d['dfs1'][0]
d2 = d['dfs2'][0]
aa, = ax11.plot(x, d1, 'r-')
bb, = ax21.plot(x, d2, 'r-')

print(d1)
print(len(d1))

try:
    while 1:
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        fig.suptitle('%s : %s,  integ = %.2f'%(name, timestamp, integ))
        
        d = con.oneshot_achilles(exposure=integ)
        d1 = d['dfs1'][0]
        d2 = d['dfs2'][0]
        aa.set_data(x, d1)
        bb.set_data(x, d2)
        matplotlib.pyplot.pause(interbal)
        print(d1[5000])
except Exception as e:
    print(e)
        
print("save last data")
numpy.save(os.path.join(savedir, '%s_dfs01.npy'%(name)), d1)
numpy.save(os.path.join(savedir, '%s_dfs02.npy'%(name)), d2)
numpy.savetxt(os.path.join(savedir, '%s_temp.txt'%(name)), [cabin_temp])
fig.savefig(os.path.join(savedir, '%s_%s.png'%(name, timestamp)))

print("end operation")

