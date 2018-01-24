#! /usr/bin/env python2

# Configurations
# ==============

# Info
# ----

name = 'oneshot_spectrometer'
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
    dirname = timestamp + '-' + memo
else:
    dirname = timestamp
    pass

savedir = "/home/amigos/data/experiment/rsky/"

# Data aquisition
# ---------------

con = ROS_controller.controller()
status = con.read_status()
cabin_temp = status.CabinTemp1

d1_list = []
d2_list = []

print('Start experimentation')

print('')

print('cabin_temp: %.2f'%(cabin_temp))

print('get spectrum...')
d = con.oneshot_achilles(exposure=integ)
#d = {"dfs1":[[100]*16384,0], "dfs2":[[200]*16384,0]}
d1 = d['dfs1'][0]
d2 = d['dfs2'][0]
d1_list.append(d1)
d2_list.append(d2)    

d1_list = numpy.array(d1_list)
d2_list = numpy.array(d2_list)

print('save...')
numpy.save(os.path.join(savedir, '%s_dfs01.npy'%(name)), d1_list)
numpy.save(os.path.join(savedir, '%s_dfs02.npy'%(name)), d2_list)
numpy.savetxt(os.path.join(savedir, '%s_temp.txt'%(name)), [cabin_temp])


x = numpy.linspace(0, 16384, len(d1_list[0]))

fig = matplotlib.pyplot.figure(figsize=(14, 5))
ax11 = fig.add_subplot(121)
ax21 = fig.add_subplot(122)
ax12 = ax11.twinx()
ax22 = ax21.twinx()

ax11.plot(x, d1_list[0], 'r-')
ax21.plot(x, d2_list[0], 'r-')

ax11.set_yscale('log')
ax21.set_yscale('log')
ax11.set_xlabel('Freq (MHz)')
ax21.set_xlabel('Freq (MHz)')
ax11.set_ylabel('Power (count)')
ax21.set_ylabel('Power (count)')
ax11.set_title('dfs01')
ax21.set_title('dfs02')

ax12.plot(x, d1_list[0], 'k.', alpha=0.2)
ax22.plot(x, d2_list[0], 'k.', alpha=0.2)
ax12.grid(True)
ax22.grid(True)
ax12.set_ylim(0, 200)
ax22.set_ylim(0, 200)
ax12.set_ylabel('Tsys (K)')
ax22.set_ylabel('Tsys (K)')


fig.suptitle('%s : %s,  integ = %.2f'%(name, timestamp, integ))
fig.savefig(os.path.join(savedir, '%s_%s.png'%(name, timestamp)))

matplotlib.pyplot.show()

