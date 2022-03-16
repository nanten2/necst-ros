#! /usr/bin/env python3
# allan.py
# -*- coding: utf-8 -*-

"""
=== About this script ===

* [Purpose and Abstract]
To carry out Allan Variance measurement.
According to Argument or default, repeat integration.
And Calcurate Allan Variance, plot result.

* [Devices]
- spectrometer (Model: AC240 - dfs01: '172.20.0.41' , 52700
                               dfs02: '172.20.0.43' , 52701)

* [How to use]
** Execute
On terminal, type like below
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

amigos@necctrl:~/RX$python allan.py --integtime <integtime> --ttl_time <total time>

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

** Input
integtime : Integration time of each measurement (Spectrometer).
            If integtime is not set, default value = (0.5 [sec]) is applied.
ttl_time  : Total time repeating measurement.
            If it is not set, default value = (500 [sec]) is applied.

** Output
- plot x 2: the result of allan Variance.

This script automatically makes a directory like below whose name are "datetime of experiment".

+++++++ Directory Structure +++++++++++
necctrl:
/home/amigos/RX/
                - allan.py
                - allan/
                        - yyyymmddHHMM/
                                       - result plot x 2
+++++++++++++++++++++++++++++++++++++++


* Written by T.Inaba

* [History]
2016/08/31 T,Inaba : ver. 1.0
"""


# Header
# ++++++
import time, sys, os


# default parameters
# ------
integtime = 0.5
# ttl_time = 500
ttl_time = 1000
# repeat = 1000
repeat = 2000
# repeat = 2

# Argument handler
# ------
import argparse

p = argparse.ArgumentParser(description="description")
p.add_argument(
    "--integtime", type=float, help="Integration time (sec). default=%.1f" % (integtime)
)
p.add_argument("--ttl_time", type=int, help="total time. default=%d" % (ttl_time))
p.add_argument(
    "--memo", type=str, help="Working memo. This will be include in directroy name."
)
args = p.parse_args()

if args.integtime is not None:
    integtime = args.integtime
if args.ttl_time is not None:
    ttl_time = args.ttl_time
if args.memo is not None:
    memo = args.memo

repeat = int(ttl_time / integtime)


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
# workdir = "/home/amigos/RX/allan/"
workdir = "/home/amigos/data/allan/"

os.mkdir(workdir + ydatetime)
filename1 = workdir + ydatetime + "/" + ydatetime


# Devices
# ------
sys.path.append("/home/amigos/rx/lib/base_param/")
import IF

# sys.path.append('/home/amigos/rx/lib/device_cntrl/')
# import TR72W
# import ac240
# import equipment_nanten
# import allantools
import ROS_controller

con = ROS_controller.controller()

att = IF.prog_att()

# dfs = equipment_nanten.dfs()#v2 comment out
# m4 = equipment_nanten.m4()#v2 comment out
# hot = equipment_nanten.hot_load()#v2 comment out

# ond = TR72W.tr72w()

# Main
# ++++++
import numpy as np
import matplotlib.pyplot as plt

# check M4, HOT and attenuator
# ------
status = con.read_status()  # v2

hot_status = status.Current_Hot  # v2
# hot_status = hot.get_status()
m4_status = status.Current_M4  # v2
# m4_status = m4.get_status()

if hot_status == "OUT":
    con.move_hot("in")
    # hot.move_r()
if m4_status == "OUT":
    con.move_m4("in")
    # m4.m4_in()

while True:  # waiting m4&hot IN
    status = con.read_status()
    hot_status = status.Current_Hot  # v2
    m4_status = status.Current_M4  # v2
    if not hot_status == "IN" and not m4_status == "IN":
        time.sleep(0.5)
        continue
    else:
        break


status = con.read_status()  # v2
hot_status = status.Current_Hot  # v2
m4_status = status.Current_M4  # v2

# hot_status = hot.get_status()
# m4_status = m4.get_status()

att_status = att.get_att()


print(" ")
print("M4 " + m4_status)
print("HOT " + hot_status)
print("IF1: " + str(att_status[0]) + " [dB]  IF2: " + str(att_status[1]) + " [dB]")
print("-----------")
print("Allan Variance measurement")
print("integration time: " + str(integtime) + " [sec]")
print("total time      : " + str(ttl_time) + " [sec]")
print("----------")
print("Then, start measurement in 1 seconds !!")
print(" ")
time.sleep(1)

# Start measurement
# ++++++
data = con.oneshot_achilles(repeat=repeat, exposure=integtime, stime=0)
# data = dfs.oneshot(repeat, integtime, 0)


data_arr1 = np.array(data["dfs1"])
print("dfs1 ", data_arr1.shape)
data_arr2 = np.array(data["dfs2"])
print("dfs2 ", data_arr2.shape)

np.savez("{}/dfs_data.npz".format(workdir + ydatetime), dfs1=data_arr1, dfs2=data_arr2)
# Calculate Allan variance
# ++++++
print(" ")
print("Now, calculating Allan Variance")
print(" ")

tau = np.linspace(1, repeat / 2 - 1, repeat / 2 - 1) * integtime
allan1 = np.array([tau])
allan2 = np.array([tau])

for ch in range(1, 17):  # for each channel
    # x1_temp = data_arr[0, :, ch * 1000 - 1]
    # x2_temp = data_arr[1, :, ch * 1000 - 1]
    x1_temp = data_arr1[:, ch * 1000]
    x2_temp = data_arr2[:, ch * 1000]
    x1_nrm = np.cumsum(x1_temp) / (np.sum(x1_temp) / repeat)
    x2_nrm = np.cumsum(x2_temp) / (np.sum(x2_temp) / repeat)
    allan1_ch = []
    allan2_ch = []
    for i in range(1, int(repeat / 2)):  # for each iteration
        sum1 = 0
        sum2 = 0
        for j in range(0, int(repeat) - 2 * i):
            temp1 = (x1_nrm[j + 2 * i] - 2 * x1_nrm[j + i] + x1_nrm[j]) ** 2
            temp2 = (x2_nrm[j + 2 * i] - 2 * x2_nrm[j + i] + x2_nrm[j]) ** 2
            sum1 = sum1 + temp1
            sum2 = sum2 + temp2

        sigma1 = 1.0 / (2 * (repeat - 2 * i) * (integtime * i) ** 2) * sum1
        sigma2 = 1.0 / (2 * (repeat - 2 * i) * (integtime * i) ** 2) * sum2
        allan1_ch.append(sigma1)
        allan2_ch.append(sigma2)

    allan1 = np.r_[allan1, np.array([allan1_ch])]
    allan2 = np.r_[allan2, np.array([allan2_ch])]

# (taus_used, adev, adeverror, adev_n) = allantools.adev(data_arr[0, :, 8000], 1./integtime, tau)
# adev_2 = np.square(adev)

# (taus2, md, mde, ns) = allantools.adev(data_arr[0, :, 8000], 1./integtime, tau)
# md_2 = np.square(md)


# Plot
# ++++++
# IF1
fig = plt.figure(figsize=(9, 9))
for i in range(0, 4):
    ax1 = fig.add_subplot(2, 2, i + 1)
    for j in range(1, 5):
        ch = (4 * i + j) * 1000
        ax1.plot(tau, allan1[4 * i + j], "+", label=str(ch) + " ch")
    ax1.set_xlabel("time (sec)")
    ax1.set_ylabel("Allan variance")
    if i == 0 or i == 1:
        ax1.set_title("IF1: Allan Variance")
    ax1.set_xlim([0.1, 1000])
    plt.xscale("log")
    plt.yscale("log")
    ax1.grid()
    ax1.legend(loc="lower left", prop={"size": 10}, numpoints=1, ncol=2)
    ax1.set_ylim([1e-6, 1e-3])
plt.savefig(filename1 + "_Allan_IF1" + str(integtime) + "s.png")
plt.show()

# IF2
fig = plt.figure(figsize=(9, 9))
for i in range(0, 4):
    ax1 = fig.add_subplot(2, 2, i + 1)
    for j in range(1, 5):
        ch = (4 * i + j) * 1000
        ax1.plot(tau, allan2[4 * i + j], "+", label=str(ch) + " ch")
    ax1.set_xlabel("time (sec)")
    ax1.set_ylabel("Allan variance")
    if i == 0 or i == 1:
        ax1.set_title("IF2: Allan Variance")
    ax1.set_xlim([0.1, 1000])
    plt.xscale("log")
    plt.yscale("log")
    ax1.grid()
    ax1.legend(loc="lower left", prop={"size": 10}, numpoints=1, ncol=2)
    ax1.set_ylim([1e-6, 1e-3])
plt.savefig(filename1 + "_Allan_IF2_integ" + str(integtime) + "s.png")
plt.show()

print(" ")
print("++++++")
print("data dir        : " + str(workdir + ydatetime) + "/")
print("total time      : " + str(ttl_time))
print("integration time: " + str(integtime))
print(
    "P ATT           : IF1 "
    + str(att_status[0])
    + ", IF2 "
    + str(att_status[1])
    + " [dB]"
)
print("++++++")


# Plot Allan tools
"""
#allantools
fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
ax1.set_xlabel("time (sec)")
ax1.set_ylabel("Allan variance")
ax1.plot(taus_used, adev_2, '+', label='8000 ch')
ax1.set_xlim([0.1, 1000])
plt.xscale('log')
plt.yscale('log')
ax1.grid()
ax1.legend(loc='lower left', prop={'size': 11}, numpoints=1)
# ax1.set_ylim([])
plt.savefig(filename1+"_Allan_IF1_tools.png")

#allantools
fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
ax1.set_xlabel("time (sec)")
ax1.set_ylabel("Allan variance")
ax1.plot(taus2, md_2, '+', label='8000 ch')
ax1.set_xlim([0.1, 1000])
plt.xscale('log')
plt.yscale('log')
ax1.grid()
ax1.legend(loc='lower left', prop={'size': 11}, numpoints=1)
# ax1.set_ylim([])
plt.savefig(filename1+"_Allan_IF1_tools_md.png")
"""
