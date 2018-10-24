#! /usr/bin/env python
# coding:utf-8
# ver_2.1

name = 'monitor_dfs_power'
description = 'Get dfs power'

# initial parameters
#---------
dt = 1
n = 60
start = 0
ts= 0
m1 = 7000
m2 = 9000
memo = ''
graph = 'on'

#set option
#---------
import argparse
p = argparse.ArgumentParser(description=description)
p.add_argument('--dt', type=float,
               help='absolute path for dt')
p.add_argument('--n', type=int,
               help='absolute path for n')
p.add_argument('--ts', type=float,
               help='absolute path for ts')
p.add_argument('--m1', type=float,
               help='absolute path for m1')
p.add_argument('--m2', type=float,
               help='absolute path for m2')
p.add_argument('--memo', type=str,
               help='Working memo. This will be include in directroy name.')
p.add_argument('--graph', type=str,
               help='Working memo. This will be include in directroy name.')

args = p.parse_args()

if args.dt is not None: dt = args.dt
if args.n is not None: n = args.n
if args.ts is not None: ts = args.ts
if args.m1 is not None: m1 = args.m1
if args.m2 is not None: m2 = args.m2
if args.graph is not None: graph = args.graph

#measurement part
#---------
import os
import time
import numpy
import math
import ROS_controller
import obs_log
list = []
list.append("--dt")
list.append(dt)
obs_log.start_script(name, list)

con = ROS_controller.controller()

datahome = '/home/amigos/data/experiment'
timestamp = time.strftime('%Y%m%d_%H%M%S',time.gmtime())
datetime = time.strftime("%Y/%m/%d - %H:%M:%S", time.gmtime())
dirname = timestamp
savedir = os.path.join(datahome, name, dirname)

print('mkdir {savedir}'.format(**locals()))
os.makedirs(savedir)

time_list = []
power1_list = []
power2_list = []

print('START MEASUREMENT')
print('')

status = con.read_status()
savetime = status.Time
num = 0
print('OVSERVATION START')
con.move_hot('in')
_time = time.time()
try:
    while num < n:
        now = time.time()
        d = con.oneshot_achilles(exposure=dt)
        d1 = d['dfs1'][0]
        d2 = d['dfs2'][0]
        power1_list.append(d1)
        power2_list.append(d2)
        time_list.append(now-_time)
        time.sleep(ts)
        num += 1
        continue
except:
    print('STOP')

print('END MEASUREMENT')
datetime_END = time.strftime("%Y/%m/%d - %H:%M:%S", time.gmtime())

f = os.path.join(savedir,'monitor_dfs_power_%s'%(timestamp))
numpy.save(f+".IF1.npy", power1_list)
numpy.save(f+".IF2.npy", power2_list)
numpy.save(f+".time.npy", time_list)

obs_log.end_script(name, dirname)

#plot part
#---------
import matplotlib.pyplot as plt
m = int(num/2)
fr = range(1, 16385)
t = time_list
average1_list = []
average2_list = []
cnt = 0
while cnt < num:
    _x = power1_list[cnt][m1-1:m2]
    _y = power2_list[cnt][m1-1:m2]
    average1 = numpy.average(_x)
    average2 = numpy.average(_y)
    average1_list.append(average1)
    average2_list.append(average2)
    cnt += 1
    continue

fig = plt.figure(figsize=(12, 9))
ax1 = fig.add_subplot(2, 2, 1)
ax2 = fig.add_subplot(2, 2, 2)
ax3 = fig.add_subplot(2, 2, 3)
ax4 = fig.add_subplot(2, 2, 4)
ax1.plot(fr, power1_list[0], "b.",label="IF1 INIT")
ax1.plot(fr, power1_list[m], "g.",label="IF1 MID")
ax1.plot(fr, power1_list[num-1], "r.",label="IF1 LAST")
ax2.plot(t, average1_list, "r.",label="IF1 AVERAGE") 
ax3.plot(fr, power2_list[0], "c.",label="IF2 INIT")
ax3.plot(fr, power2_list[m], "m.",label="IF2 MID")
ax3.plot(fr, power2_list[num-1], "y.",label="IF2 LAST")
ax4.plot(t, average2_list, "b.",label="IF2 AVERAGE") 
ax1.set_title("DFS1 band character")
ax2.set_title("DFS1 AVERAGE")
ax3.set_title("DFS2 band character")
ax4.set_title("DFS2 AVERAGE")
ax1.set_xlabel("channel")
ax1.set_ylabel("count")
ax1.set_xlim([0, 16384])
ax1.grid(linestyle='--')
ax1.legend(numpoints=1, prop={'size': 10})
ax1.annotate(str(datetime)+"[UT]", xy=(0.05, 0.95), xycoords='axes fraction', fontsize=12,
             horizontalalignment='left', verticalalignment='bottom')
ax1.annotate(str(datetime_END)+"[UT]", xy=(0.05, 0.90), xycoords='axes fraction', fontsize=12,
             horizontalalignment='left', verticalalignment='bottom')
ax2.set_xlabel("time [sec]")
ax2.set_ylabel("count")
ax2.grid(linestyle='--')
ax2.legend(numpoints=1, prop={'size': 10})
ax2.annotate('m1='+str(m1), xy=(0.05, 0.95), xycoords='axes fraction', fontsize=12,
             horizontalalignment='left', verticalalignment='bottom')
ax2.annotate('m2='+str(m2), xy=(0.05, 0.90), xycoords='axes fraction', fontsize=12,
             horizontalalignment='left', verticalalignment='bottom')
ax3.set_xlabel("channel")
ax3.set_ylabel("count")
ax3.set_xlim([0, 16384])
ax3.grid(linestyle='--')
ax3.legend(numpoints=1, prop={'size': 10})
ax3.annotate('dt='+str(dt), xy=(0.05, 0.95), xycoords='axes fraction', fontsize=12,
             horizontalalignment='left', verticalalignment='bottom')
ax3.annotate('n='+str(n), xy=(0.05, 0.90), xycoords='axes fraction', fontsize=12,
             horizontalalignment='left', verticalalignment='bottom')
ax3.annotate('ts='+str(ts), xy=(0.05, 0.85), xycoords='axes fraction', fontsize=12,
             horizontalalignment='left', verticalalignment='bottom')
ax4.set_xlabel("time [sec]")
ax4.set_ylabel("count")
fig.tight_layout()
ax4.grid(linestyle='--')
ax4.legend(numpoints=1, prop={'size': 10})
#print(f)
#print(timestamp)
#import sys
#sys.path.append("/home/amigos/NECST/script/data/monitor_dfs_power/"+timestamp+'/')
plt.savefig(f+'.png')
if graph == 'on':
    #plt.show()
    pass
else:
    pass
