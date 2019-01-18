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
os.mkdir(workdir+ydatetime+"_IF1")
os.mkdir(workdir+ydatetime+"_IF2")
filename1 = workdir+ydatetime+"_IF1"+"/"+ydatetime
filename2 = workdir+ydatetime+"_IF2"+"/"+ydatetime

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
    #data = dfs.oneshot(1, integtime, 0)
    data = con.oneshot_achilles(repeat = 1, exposure=integtime, stime=0)
    print(sa1.shape)
    print("###",np.array(data['dfs1'][0]).shape)
    sa1 = np.r_[sa1[0], np.array(data['dfs1'][0])]
    sa2 = np.r_[sa2[0], np.array(data['dfs2'][0])]
    #sa1 = np.r_[sa1, data[0]]
    #sa2 = np.r_[sa2, data[1]]
    sa1_log = np.r_[sa1_log[0], 10*np.log10(data['dfs1'][0])]
    sa2_log = np.r_[sa2_log[0], 10*np.log10(data['dfs2'][0])]
    print(str(i)+"dB finished")
np.savetxt(filename1+".csv", sa1, delimiter=",")
np.savetxt(filename2+".csv", sa2, delimiter=",")

print(" ")
print("Finish measurement")
print("Now plotting...")

# Plot
# ++++++

# plot raw data
# ------
fig12 = plt.figure()
ax1 = fig12.add_subplot(1, 1, 1)
for i in range(12):
    ax1.plot(sa1[0], sa1[i+1], label=str(i)+"[dB]")
ax1.set_xlabel("Ch")
ax1.set_ylabel("Count")
ax1.set_title(ydatetime+"_IF1: raw data")
ax1.set_xlim([0, 16383])
ax1.set_ylim([0, 3*np.mean(sa1[2])])
ax1.grid()
ax1.legend(bbox_to_anchor=(1.01, 1.012), loc='upper left', prop={'size': 13})
plt.subplots_adjust(left=0.1, right=0.8)
plt.savefig(filename1+"_IF1_raw_data_plot.png")

fig13 = plt.figure()
ax2 = fig13.add_subplot(1, 1, 1)
for i in range(12):
    ax2.plot(sa2[0], sa2[i+1], label=str(i)+"[dB]")
ax2.set_xlabel("Ch")
ax2.set_ylabel("Count")
ax2.set_title(ydatetime+"_IF2: raw data")
ax2.set_xlim([0, 16383])
ax2.set_ylim([0, 3*np.mean(sa2[2])])
ax2.grid()
ax2.legend(bbox_to_anchor=(1.01, 1.012), loc='upper left', prop={'size': 13})
plt.subplots_adjust(left=0.1, right=0.8)
plt.savefig(filename2+"_IF2_raw_data_plot.png")

fig12 = plt.figure()
ax1 = fig12.add_subplot(1, 1, 1)
for i in range(12):
    ax1.plot(sa1[0], sa1_log[i+1], label=str(i)+"[dB]")
ax1.set_xlabel("Ch")
ax1.set_ylabel("10*log10(count)")
ax1.set_title(ydatetime+"_IF1: raw data")
ax1.set_xlim([0, 16383])
#ax1.set_ylim([0, 6000])
ax1.grid()
ax1.legend(bbox_to_anchor=(1.01, 1.012), loc='upper left', prop={'size': 13})
plt.subplots_adjust(left=0.1, right=0.8)
plt.savefig(filename1+"_IF1_raw_data_plot_log.png")

fig13 = plt.figure()
ax2 = fig13.add_subplot(1, 1, 1)
for i in range(12):
    ax2.plot(sa2[0], sa2_log[i+1], label=str(i)+"[dB]")
ax2.set_xlabel("Ch")
ax2.set_ylabel("10*log10(count)")
ax2.set_title(ydatetime+"_IF2: raw data")
ax2.set_xlim([0, 16383])
#ax2.set_ylim([0, 6000])
ax2.grid()
ax2.legend(bbox_to_anchor=(1.01, 1.012), loc='upper left', prop={'size': 13})
plt.subplots_adjust(left=0.1, right=0.8)
plt.savefig(filename2+"_IF2_raw_data_plot_log.png")


#plot linearity measurement
# ------
results1 = []
for i in range(1, 12):   #sa[1]-[11] = 0-10 [dB]
    fig = plt.figure()
    ax1 = fig.add_subplot(1, 1, 1)
    for k in range(i+1, 13):  #i+1-12
        ax1.plot(sa1[0], sa1_log[k]-sa1_log[i], label=str(k-1)+"[dB]")
    ax1.set_xlabel("Ch")
    ax1.set_ylabel("dB scale count variation against reference")
    ax1.set_title(ydatetime+"_IF1: ref = "+str(i-1)+"[dB]")
    ax1.set_xlim([0, 16383])
    ax1.set_ylim([-6.5, 0.25])
    ax1.grid()
    for j in range(40):
        plt.axhline(y=-j*0.25, lw=0.5, linestyle="--", color='black')
    ax1.legend(bbox_to_anchor=(1.01, 1.012), loc='upper left', prop={'size': 13})
    plt.subplots_adjust(left=0.1, right=0.8)
    plt.savefig(filename1+"_IF1_"+str(i-1)+"dB.png")
"""
    ave = np.average(sa1[i+1]-sa1[1])
    results1.extend([ave, ave+i])
    print "%d dBm -- %f , %f" %(i, ave, ave+i)
"""

results2 = []
for i in range(1, 12):   #sa[1]-[11] = 0-10 [dB]
    fig = plt.figure()
    ax1 = fig.add_subplot(1, 1, 1)
    for k in range(i+1, 13):  #i+1-12
        ax1.plot(sa1[0], sa2_log[k]-sa2_log[i], label=str(k-1)+"[dB]")
    ax1.set_xlabel("Ch")
    ax1.set_ylabel("dB scale count variation against reference")
    ax1.set_title(ydatetime+"_IF2: ref = "+str(i-1)+"[dB]")
    ax1.set_xlim([0, 16383])
    ax1.set_ylim([-6.5, 0.25])
    ax1.grid()
    for j in range(40):
        plt.axhline(y=-j*0.25, lw=0.5, linestyle="--", color='black')
    ax1.legend(bbox_to_anchor=(1.01, 1.012), loc='upper left', prop={'size': 13})
    plt.subplots_adjust(left=0.1, right=0.8)
    plt.savefig(filename2+"_IF2_"+str(i-1)+"dB.png")
"""
    ave = np.average(sa2[i+1]-sa2[1])
    results2.extend([ave, ave+i])
    print "%d dBm -- %f , %f" %(i, ave, ave+i)
"""

"""
# Histogram
# ------
# IF1
for i in range(0, 9):  # reference att = 0--6 dB
    fig = plt.figure(figsize=(12, 9))
    if i >= 7:
        for j in range(i+1, i+5):
            if j >= 12:
                break
            ax1 = fig.add_subplot(2, 2, j-i)
            ax1.hist(j-i+sa1_log[j+1, 4000:12000]-sa1_log[i+1, 4000:12000])
            hist1_std = np.std(j-i+sa1_log[j+1, 4000:12000]-sa1_log[i+1, 4000:12000])
            ax1.set_title("(ref, att) = ("+str(i)+", "+str(j)+") dB", )
            ax1.set_xlabel("difference [dB]")
            ax1.set_xlim([-0.2, 0.2])
            ax1.annotate("std = "+str('%.3f' % hist1_std), xy=(0.65, 0.85), xycoords='axes fraction', fontsize=10,
             horizontalalignment='left', verticalalignment='bottom')
            plt.locator_params(axis='x', tight=True, nbins=4)
            plt.savefig(filename1+"_IF1_hist_"+str(i)+"dB.png")

    else:
        for j in range(i+1, i+7):  # att = ref+1 -- ref+6
            if j >= 12:
                break
            ax1 = fig.add_subplot(2, 3, j-i)
            ax1.hist(j-i+sa1_log[j+1, 4000:12000]-sa1_log[i+1, 4000:12000])
            hist1_std = np.std(j-i+sa1_log[j+1, 4000:12000]-sa1_log[i+1, 4000:12000])
            ax1.set_title("(ref, att) = ("+str(i)+", "+str(j)+") dB", )
            ax1.set_xlabel("difference [dB]")
            ax1.set_xlim([-0.2, 0.2])
            ax1.annotate("std = "+str('%.3f' % hist1_std), xy=(0.65, 0.85), xycoords='axes fraction', fontsize=10,
             horizontalalignment='left', verticalalignment='bottom')
            plt.locator_params(axis='x', tight=True, nbins=4)
            plt.savefig(filename1+"_IF1_hist_"+str(i)+"dB.png")

# IF2
for i in range(0, 9):  # reference att = 0--6 dB
    fig = plt.figure(figsize=(12, 9))
    if i >= 7:
        for j in range(i+1, i+5):
            if j >= 12:
                break
            ax1 = fig.add_subplot(2, 2, j-i)
            ax1.hist(j-i+sa2_log[j+1, 4000:12000]-sa2_log[i+1, 4000:12000], bins=32, histtype="stepfilled")
            hist2_std = np.std(j-i+sa1_log[j+1, 4000:12000]-sa1_log[i+1, 4000:12000])
            ax1.set_title("(ref, att) = ("+str(i)+", "+str(j)+") dB", )
            ax1.set_xlabel("difference [dB]")
            ax1.set_xlim([-0.2, 0.2])
            ax1.annotate("std = "+str('%.3f' % hist2_std), xy=(0.65, 0.85), xycoords='axes fraction', fontsize=10,
             horizontalalignment='left', verticalalignment='bottom')
            plt.locator_params(axis='x', tight=True, nbins=4)
            plt.savefig(filename2+"_IF2_hist_"+str(i)+"dB.png")

    else:
        for j in range(i+1, i+7):  # att = ref+1 -- ref+6
            if j >= 12:
                break
            ax1 = fig.add_subplot(2, 3, j-i)
            ax1.hist(j-i+sa2_log[j+1, 4000:12000]-sa2_log[i+1, 4000:12000], bins=32, histtype="stepfilled")
            hist2_std = np.std(j-i+sa1_log[j+1, 4000:12000]-sa1_log[i+1, 4000:12000])
            ax1.set_title("(ref, att) = ("+str(i)+", "+str(j)+") dB", )
            ax1.set_xlabel("difference [dB]")
            ax1.set_xlim([-0.2, 0.2])
            ax1.annotate("std = "+str('%.3f' % hist2_std), xy=(0.65, 0.85), xycoords='axes fraction', fontsize=10,
             horizontalalignment='left', verticalalignment='bottom')
            plt.locator_params(axis='x', tight=True, nbins=4)
            plt.savefig(filename2+"_IF2_hist_"+str(i)+"dB.png")
"""

# fit plot
# ------
x = np.linspace(0, 12, 12)
y = - x
for i in range(0, 9):
    fig = plt.figure(figsize=(12, 12))
    patt = np.linspace(1, 11-i, 11-i)

    for j in range(5, 13, 2):  # j * 1000 = channel
        pow = []
        var = []

        for k in range(i+1, 12):  # Making data array
            pow.append(np.median(sa1_log[k+1, j*1000-5:j*1000+5] - sa1_log[i+1, j*1000-5:j*1000+5]))
#            var.append(np.var(sa1_log[k+1, j*1000-5:j*1000+5] - sa1_log[i+1, j*1000-5:j*1000+5]))

        ax1 = fig.add_subplot(2, 2, (j-3)/2)
        ax2 = ax1.twinx()
#        plt.errorbar(patt, pow, yerr=var, fmt='ro', ecolor='r')
        ax1.plot(patt, pow, "bo", label="power")
        ax1.plot(x, y, 'r-')
        ax1.set_title("IF1: (ref, ch) = ("+str(i)+" dB, "+str(j*1000)+" ch)", )
        ax1.set_xlabel("Prog ATT [dB]")
        if (j-3)/2 == 1 or (j-3)/2 == 3:
            ax1.set_ylabel("dB scale count variation against reference")
        ax1.set_xlim([0, patt[-1]+0.5])
        ax1.set_ylim([-patt[-1]-0.5, 0])

        ax2.plot(patt, pow + patt, "go", label="difference from linear")
        if (j-3)/2 == 2 or (j-3)/2 == 4:
            ax2.set_ylabel("difference from linear")
        ax2.set_ylim([-0.25, 0.25])
        ax1.hlines(y=-x, lw=0.5, linestyle="--", color='black', xmin=0, xmax=11)
        plt.vlines(x=x, lw=0.5, linestyle="--", color='black', ymin=-10, ymax=0)
        ax1.grid()
        #plt.legend(numpoints=1)
    plt.savefig(filename1+"_IF1_fit_"+str(i)+"dB.png")

for i in range(0, 9):
    fig = plt.figure(figsize=(12, 12))
    patt = np.linspace(1, 11-i, 11-i)

    for j in range(5, 13, 2):  # j * 1000 = channel
        pow2 = []
        var2 = []

        for k in range(i+1, 12):  # Making data array
            pow2.append(np.median(sa2_log[k+1, j*1000-5:j*1000+5] - sa2_log[i+1, j*1000-5:j*1000+5]))
#            var2.append(np.var(sa2_log[k+1, j*1000-5:j*1000+5] - sa2_log[i+1, j*1000-5:j*1000+5]))

        ax1 = fig.add_subplot(2, 2, (j-3)/2)
        ax2 = ax1.twinx()
#       plt.errorbar(patt, pow2, yerr=var2, fmt='ro', ecolor='r')
        ax1.plot(patt, pow2, "bo", label="power")
        ax1.plot(x, y, 'r-')
        ax1.set_title("IF2: (ref, ch) = ("+str(i)+" dB, "+str(j*1000)+" ch)", )
        ax1.set_xlabel("Prog ATT [dB]")
        if (j-3)/2 == 1 or (j-3)/2 == 3:
            ax1.set_ylabel("dB scale count variation against reference")
        ax1.set_xlim([0, patt[-1]+0.5])
        ax1.set_ylim([-patt[-1]-0.5, 0])

        ax2.plot(patt, pow2 + patt, "go", label="difference from linear")
        if (j-3)/2 == 2 or (j-3)/2 == 4:
            ax2.set_ylabel("difference from linear")
        ax2.set_ylim([-0.25, -0.25])
        ax1.hlines(y=-x, lw=0.5, linestyle="--", color='black', xmin=0, xmax=11)
        plt.vlines(x=x, lw=0.5, linestyle="--", color='black', ymin=-10, ymax=0)
        ax1.grid()
        #plt.legend(numpoints=1)
    plt.savefig(filename2+"_IF2_fit_"+str(i)+"dB.png")





print("Finish plotting")

att.set_att(att_status[0], att_status[1])
time.sleep(1)
att_status_2 = att.get_att()

print("P ATT setting")
print("IF1: "+str(att_status_2[0])+" [dB]  IF2: "+str(att_status_2[1])+" [dB]")
