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
import matplotlib.pyplot
import datetime
matplotlib.pyplot.rcParams['font.size'] = 9

import sys
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")
import ROS_controller


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

d1_list = []
d2_list = []

print('Start experimentation')

print('')

print('R')
con.move_hot('in')

time.sleep(1)

print('cabin_temp: %.2f'%(cabin_temp))

print('get spectrum...')
d = con.oneshot_achilles(exposure=integ)
#d = {"dfs1":[[100]*16384,0], "dfs2":[[200]*16384,0]}
d1 = d['dfs1'][0]
d2 = d['dfs2'][0]
d1_list.append(d1)
d2_list.append(d2)    

print('SKY')
con.move_hot('out')

time.sleep(1)

print('get spectrum...')
d = con.oneshot_achilles(exposure=integ)
#d = {"dfs1":[[10]*16384,0], "dfs2":[[20]*16384,0]}
d1 = d['dfs1'][0]
d2 = d['dfs2'][0]
d1_list.append(d1)
d2_list.append(d2)    


con.move_hot('in')

d1_list = numpy.array(d1_list)
d2_list = numpy.array(d2_list)



print('save...')
numpy.save(os.path.join(savedir, '%s_dfs01.npy'%(name)), d1_list)
numpy.save(os.path.join(savedir, '%s_dfs02.npy'%(name)), d2_list)
numpy.savetxt(os.path.join(savedir, '%s_temp.txt'%(name)), [cabin_temp])


def tsys(dhot, dsky, thot):
    dhot = numpy.array(dhot)
    dsky = numpy.array(dsky)
    
    y = dhot / dsky
    tsys = thot / (y - 1.)
    return tsys

tsys1 = tsys(d1_list[0], d1_list[1], cabin_temp)
tsys2 = tsys(d2_list[0], d2_list[1], cabin_temp)

d1_av = numpy.mean(d1_list[:,500:-500], axis=1)
d2_av = numpy.mean(d2_list[:,500:-500], axis=1)
tsys1_av = tsys(d1_av[0], d1_av[1], cabin_temp)
tsys2_av = tsys(d2_av[0], d2_av[1], cabin_temp)

x = numpy.linspace(0, 1000, len(d1_list[0]))

fig = matplotlib.pyplot.figure(figsize=(14, 5))
ax11 = fig.add_subplot(121)
ax21 = fig.add_subplot(122)
ax12 = ax11.twinx()
ax22 = ax21.twinx()

ax11.plot(x, d1_list[0], 'r-')
ax21.plot(x, d2_list[0], 'r-')
ax11.plot(x, d1_list[1], 'b-')
ax21.plot(x, d2_list[1], 'b-')

ax11.set_yscale('log')
ax21.set_yscale('log')
ax11.set_xlabel('Freq (MHz)')
ax21.set_xlabel('Freq (MHz)')
ax11.set_ylabel('Power (count)')
ax21.set_ylabel('Power (count)')
ax11.set_title('dfs01')
ax21.set_title('dfs02')

ax12.plot(x, tsys1, 'k.', alpha=0.2)
ax22.plot(x, tsys2, 'k.', alpha=0.2)
ax12.grid(True)
ax22.grid(True)
ax12.set_ylim(0, 200)
ax22.set_ylim(0, 200)
ax12.set_ylabel('Tsys (K)')
ax22.set_ylabel('Tsys (K)')

ax11.text(0.05, 0.9, 'Tsys = %.1f'%(tsys1_av), transform=ax11.transAxes)
ax21.text(0.05, 0.9, 'Tsys = %.1f'%(tsys2_av), transform=ax21.transAxes)

fig.suptitle('%s : %s,  integ = %.2f'%(name, timestamp, integ))
fig.savefig(os.path.join(savedir, '%s_%s.png'%(name, timestamp)))

matplotlib.pyplot.show()


