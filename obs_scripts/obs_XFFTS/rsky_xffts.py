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
plt.rcParams['font.size'] = 12

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
savedir = "/home/amigos/data/rsky/"+day+"/"
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
data_list = []
data = d.oneshot(integ,1,time.time()+1)[2][0]
data_list.append(data)

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
data = d.oneshot(integ,1,time.time()+1)[2][0]
data_list.append(data)


con.move_hot('in')

def tsys(dhot, dsky, thot):
    y = dhot / dsky
    tsys = thot / (y - 1.)
    return tsys

#memo
#datalist[0-1]#hot or sky
#datalist[*][0-20] : IF1-20 32768ch
data_list = numpy.array(data_list)
print(numpy.shape(data_list))
data_list = numpy.swapaxes(data_list, 0, 1)
print(numpy.shape(data_list))
for i in range(4):
    d1_list = data_list[i * 4]
    d2_list = data_list[1 + i*4]
    d3_list = data_list[2 + i*4]
    d4_list = data_list[3 + i*4]

    tsys1 = tsys(d1_list[0], d1_list[1], cabin_temp)
    tsys2 = tsys(d2_list[0], d2_list[1], cabin_temp)
    tsys3 = tsys(d3_list[0], d3_list[1], cabin_temp)
    tsys4 = tsys(d4_list[0], d4_list[1], cabin_temp)
    
    d1_av = numpy.mean(d1_list[:,500:-500], axis=1)
    d2_av = numpy.mean(d2_list[:,500:-500], axis=1)
    d3_av = numpy.mean(d3_list[:,500:-500], axis=1)
    d4_av = numpy.mean(d4_list[:,500:-500], axis=1)

    tsys1_av = tsys(d1_av[0], d1_av[1], cabin_temp)
    tsys2_av = tsys(d2_av[0], d2_av[1], cabin_temp)
    tsys3_av = tsys(d3_av[0], d3_av[1], cabin_temp)
    tsys4_av = tsys(d4_av[0], d4_av[1], cabin_temp)

    x = numpy.linspace(0, 2000, len(d1_list[0]))#XFFTS bw = 0-2000MHz
    
    fig, ax = plt.subplots(2, 2, figsize = (14,10))

    ax[0,0].plot(x, d1_list[0], 'r-', label="HOT")
    ax[0,1].plot(x, d2_list[0], 'r-', label="HOT")
    ax[1,0].plot(x, d3_list[0], 'r-', label="HOT")
    ax[1,1].plot(x, d4_list[0], 'r-', label="HOT")

    ax[0,0].plot(x, d1_list[0], 'b-', label="SKY")
    ax[0,1].plot(x, d2_list[0], 'b-', label="SKY")
    ax[1,0].plot(x, d3_list[0], 'b-', label="SKY")
    ax[1,1].plot(x, d4_list[0], 'b-', label="SKY")

    ax00 = ax[0,0].twinx()
    ax00.plot(x, tsys1, "g.", label="Tsys")
    ax01 = ax[0,1].twinx()
    ax01.plot(x, tsys2, "g.", label="Tsys")
    ax10 = ax[1,0].twinx()
    ax10.plot(x, tsys3, "g.", label="Tsys")
    ax11 = ax[1,1].twinx()
    ax11.plot(x, tsys4, "g.", label="Tsys")

    ax[0,0].legend()
    ax[0,1].legend()
    ax[1,0].legend()
    ax[1,1].legend()
    ax00.legend()
    ax01.legend()
    ax10.legend()
    ax11.legend()

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

    ax00.text(0.05, 0.9, 'Tsys = %.1f'%(tsys1_av), transform=ax00.transAxes)
    ax01.text(0.05, 0.9, 'Tsys = %.1f'%(tsys2_av), transform=ax01.transAxes)
    ax10.text(0.05, 0.9, 'Tsys = %.1f'%(tsys3_av), transform=ax10.transAxes)
    ax11.text(0.05, 0.9, 'Tsys = %.1f'%(tsys4_av), transform=ax11.transAxes)

    ax[0,0].grid(True)
    ax[0,1].grid(True)
    ax[1,0].grid(True)
    ax[1,1].grid(True)
    
    plt.tight_layout()
    fig.suptitle('%s : %s - %s/4,  integ = %.2f'%(name, timestamp, i+1, integ))
    fig.savefig(os.path.join(savedir, '{}_{}_{}.png'.format(name, timestamp, i)))
    plt.show()
