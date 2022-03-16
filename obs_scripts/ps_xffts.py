#! /usr/bin/env python3
# coding:utf-8

import signal
import ROS_controller
import doppler_nanten
import numpy
import time
import os
import argparse
from datetime import datetime
import logger
import sys
import read_obsfile
import shutil
import log_weather

# Configurations
# ==============
# Info
# ----

name = "position_switching2019"
description = "Get P/S spectrum"


# Default parameters
# ------------------
obsfile = ""
tau = 0.0

# setup logger
# ===========
now = datetime.utcnow()
log_path = "/home/amigos/log/{}.txt".format(now.strftime("%Y%m%d"))
logger = logger.logger(__name__, filename=log_path)
log = logger.setup_logger()
logger.obslog(sys.argv)
start_time = time.time()

# Argument handler
# ================


p = argparse.ArgumentParser(description=description)
p.add_argument("--obsfile", type=str, help="absolute path for obsfile")
p.add_argument("--tau", type=float, help="tau. default=%.1f" % (tau))

args = p.parse_args()

if args.obsfile is not None:
    obsfile = args.obsfile
if args.tau is not None:
    tau = args.tau

# Read Observation file
# =====================
obsdir = "/home/amigos/necst-obsfiles/"
try:
    obs = read_obsfile.read(os.path.join(obsdir, obsfile))
except Exception as e:
    log.exception(e)
    sys.exit()

# Save file
# =========
datahome = "/home/amigos/hdd/data/"
timestamp = time.strftime("%Y%m%d%H%M%S")
dirname = "n%s_%s_%s_otf_%s" % (
    timestamp,
    obs["molecule_1"],
    obs["transiti_1"].split("=")[1],
    obs["object"],
)
savedir = os.path.join(datahome, name, dirname)
log.info("mkdir {savedir}".format(**locals()))
os.makedirs(savedir)
logger.obslog("savedir : {}".format(savedir), lv=1)

dirname = "n{}_{}_{}_otf_{}".format(
    now.strftime("%Y%m%d%H%M%S"),
    obs["molecule_1"],
    obs["transiti_1"].split("=")[1],
    obs["object"],
)
xffts_datapath = os.path.join(savedir, "xffts.ndf")

log.debug("obsdir : {}".format(obsdir))
log.debug("log_path : {}".format(log_path))
log.debug("dirname : {}".format(dirname))
log.debug("xffts : {}".format(xffts_datapath))

# copy hosei & obsfiles
# =====================
shutil.copy("/home/amigos/ros/src/necst/lib/hosei_230.txt", savedir)
shutil.copy(os.path.join(obsdir, obsfile), savedir)

# Main
# ====
# Preparetion
# ------------------
dp = doppler_nanten.doppler_nanten()

con = ROS_controller.controller()
con.dome_track()
con.move_stop()

### Logging Start
con.pub_loggerflag(savedir)


def handler(num, flame):
    print("!!ctrl+C!!")
    print("STOP MOVING")
    con.move_stop()
    con.dome_stop()
    con.pub_loggerflag("")
    con.obs_status(active=False)
    sys.exit()


signal.signal(signal.SIGINT, handler)

# setup weather logger
# ====================
logw = log_weather.Weather_log(os.path.join(savedir, "weather.csv"))
logw.initialize()


def save_weatherlog(scan_num, obs_mode):
    d = con.read_status()
    logw.write(
        time.time(),
        d.InTemp,
        d.OutTemp,
        d.InHumi,
        d.OutHumi,
        d.WindDir,
        d.WindSp,
        d.Press,
        d.Rain,
        d.CabinTemp1,
        d.CabinTemp2,
        d.DomeTemp1,
        d.DomeTemp2,
        d.GenTemp1,
        d.GenTemp2,
        scan_num,
        obs_mode,
    )
    log.info("Saved weather log")


# Use obs_file
# ------------------

offset_Az = obs["offset_Az"]
offset_El = obs["offset_El"]
vlsr = obs["vlsr"]

lambda_on = obs["lambda_on"]  # on
beta_on = obs["beta_on"]  # on
lambda_off = obs["lambda_off"]  # off
beta_off = obs["beta_off"]  # off
coordsys = obs["coordsys"].lower()  # coord
dcos = obs["otadel"]  # dcos
integ_on = obs["exposure"]  # on iinteg
integ_off = obs["exposure_off"]  # off integ

lamdel_on = obs["lamdel"]  # offset_on
betdel_on = obs["betdel"]  # offset_on
lamdel_off = obs["lamdel_off"]  # offset_off
betdel_off = obs["betdel_off"]  # offset_off
cosydel = obs["cosydel"].lower()  # offset_coord
offset_dcos = obs["otadel_off"]  # offset_dcos

if dcos.lower() == "y":
    dcos = 1
else:
    dcos = 0
if offset_dcos.lower() == "y":
    off_dcos = 1
else:
    off_dcos = 0

if obs["lo1st_sb_1"] == "U":  # 後半に似たのがあるけど気にしない()
    sb1 = 1
else:
    sb1 = -1
if obs["lo1st_sb_2"] == "U":  # 後半に似たのがあるけど気にしない()
    sb2 = 1
else:
    sb2 = -1


datahome = "/home/amigos/data/"
timestamp = time.strftime("%Y%m%d_%H%M%S")
dirname = timestamp
savedir = os.path.join(datahome, name, dirname)

log.info("mkdir {savedir}".format(**locals()))
os.makedirs(savedir)

log.info("Start experimentation")
con.pub_encdb_flag(True, os.path.join(savedir, "enc.db"))
savetime = con.read_status().Time
num = 0
n = int(obs["nSeq"])
latest_hottime = 0

con.obs_status(
    active=True,
    obsmode=obs["obsmode"],
    obs_script=__file__,
    obs_file=obsfile,
    target=obs["object"],
    num_on=obs["nON"],
    num_seq=obs["nSeq"],
    exposure_hot=obs["exposure_off"],
    exposure_off=obs["exposure_off"],
    exposure_on=obs["exposure"],
)
while num < n:
    log.info("observation :" + str(num + 1) + "\n")
    save_weatherlog(num, "")
    con.onepoint_move(
        lambda_off,
        beta_off,
        coordsys,
        off_x=lamdel_off,
        off_y=betdel_off,
        offcoord=cosydel,
        dcos=dcos,
    )
    con.antenna_tracking_check()
    con.dome_tracking_check()
    log.info("tracking OK")

    _now = time.time()
    if _now > latest_hottime + 60 * obs["load_interval"]:
        print("R" + "\n")

        con.move_hot("in")
        status = con.read_status()
        while status.Current_Hot != "IN":
            log.debug("wait hot_move...")
            status = con.read_status()
            time.sleep(0.5)
        con.obs_status(active=True, current_num=num, current_position="HOT")

        log.info("get spectrum...")
        dp1 = dp.set_track(
            lambda_on,
            beta_on,
            vlsr,
            coordsys,
            lamdel_on,
            betdel_on,
            dcos,
            cosydel,
            integ_off * 2,
            obs["restfreq_1"] / 1000.0,
            obs["restfreq_2"] / 1000.0,
            sb1,
            sb2,
            8038.000000000 / 1000.0,
            9301.318999999 / 1000.0,
        )
        time.sleep(integ_off)

        status = con.read_status()
        temp = float(status.CabinTemp1)  # + 273.15
        # d = con.oneshot_achilles(exposure=integ_off)# AC240
        con.xffts_publish_flag(scan_num=num, obs_mode="HOT")  # OFF? HOT?
        time.sleep(integ_off)
        con.xffts_publish_flag()  # OFF? HOT?
        # d1 = d['dfs1'][0]
        # d2 = d['dfs2'][0]
        # d1_list.append(d1)
        # d2_list.append(d2)
        # #tdim6_list.append([16384, 1, 1])
        # tmp_time = status.Time
        # tmp2 = datetime.fromtimestamp(tmp_time)
        # tmp3 = tmp2.strftime("%Y/%m/%d %H:%M:%S")
        # date_list.append(tmp3)
        # thot_list.append(temp)
        # vframe_list.append(dp1[0])
        # vframe2_list.append(dp1[0])
        # lst_list.append(status.LST)
        # az_list.append(status.Current_Az)
        # el_list.append(status.Current_El)
        # tau_list.append(tau)
        # hum_list.append(status.OutHumi)
        # tamb_list.append(status.OutTemp)
        # press_list.append(status.Press)
        # windspee_list.append(status.WindSp)
        # winddire_list.append(status.WindDir)
        # sobsmode_list.append('HOT')
        # mjd_list.append(status.MJD)
        # secofday_list.append(status.Secofday)
        # subref_list.append(status.Current_M2)
        # latest_hottime = time.time()
        # P_hot = numpy.sum(d1)
        # tsys_list.append(0)
        # _2NDLO_list1.append(dp1[3]['sg21']*1000)
        # _2NDLO_list2.append(dp1[3]['sg22']*1000)
        # print("sg21 : ", dp1[3]['sg21']*1000)
        # print("sg22 : ", dp1[3]['sg22']*1000, "\n")

        pass

    else:
        # dp1 = dp.set_track(lambda_on, beta_on, vlsr, coordsys, lamdel_on, betdel_on, dcos, cosydel,
        # integ_off, obs['restfreq_1']/1000., obs['restfreq_2']/1000.,
        # sb1, sb2, 8038.000000000/1000., 9301.318999999/1000.)
        pass
    print("OFF" + "\n")
    con.move_hot("out")
    status = con.read_status()
    while status.Current_Hot != "OUT":
        print("wait hot_move...")
        status = con.read_status()
        time.sleep(0.5)
    con.obs_status(active=True, current_num=num, current_position="OFF")
    print("get spectrum...")
    # con.observation("start", integ_off)# getting one_shot_data
    time.sleep(integ_off)

    status = con.read_status()
    temp = float(status.CabinTemp1)  # + 273.15
    # d = con.oneshot_achilles(exposure=integ_off)
    con.xffts_publish_flag(scan_num=num, obs_mode="OFF")  # OFF
    time.sleep(integ_off)
    con.xffts_publish_flag()  # OFF
    # d = {'dfs1': [[1]*16384,1], 'dfs2': [[10]*16384,11]}
    # d1 = d['dfs1'][0]
    # d2 = d['dfs2'][0]
    # d1_list.append(d1)
    # d2_list.append(d2)
    # tdim6_list.append([16384, 1, 1])
    # tmp_time = status.Time
    # tmp2 = datetime.fromtimestamp(tmp_time)
    # tmp3 = tmp2.strftime("%Y/%m/%d %H:%M:%S")
    # date_list.append(tmp3)
    # thot_list.append(temp)
    # vframe_list.append(dp1[0])
    # vframe2_list.append(dp1[0])
    # lst_list.append(status.LST)
    # az_list.append(status.Current_Az)
    # el_list.append(status.Current_El)
    # tau_list.append(tau)
    # hum_list.append(status.OutHumi)
    # tamb_list.append(status.OutTemp)
    # press_list.append(status.Press)
    # windspee_list.append(status.WindSp)
    # winddire_list.append(status.WindDir)
    # sobsmode_list.append('OFF')
    # mjd_list.append(status.MJD)
    # secofday_list.append(status.Secofday)
    # subref_list.append(status.Current_M2)
    # P_sky = numpy.sum(d1)
    # tsys = temp/(P_hot/P_sky-1)
    # tsys_list.append(tsys)
    # _2NDLO_list1.append(dp1[3]['sg21']*1000)
    # _2NDLO_list2.append(dp1[3]['sg22']*1000)
    # print("sg21 : ", dp1[3]['sg21']*1000)
    # print("sg22 : ", dp1[3]['sg22']*1000, "\n")
    log.info("move ON")

    con.onepoint_move(
        lambda_on,
        beta_on,
        coordsys,
        off_x=lamdel_on,
        off_y=betdel_on,
        offcoord=cosydel,
        dcos=dcos,
    )

    con.obs_status(active=True, current_num=num, current_position="ON")
    con.antenna_tracking_check()
    con.dome_tracking_check()
    log.info("tracking OK")

    log.info("get spectrum...")

    status = con.read_status()
    temp = float(status.CabinTemp1)  # + 273.15
    # d = con.oneshot_achilles(exposure=integ_on)
    con.xffts_publish_flag(scan_num=num, obs_mode="ON")
    time.sleep(integ_on)
    con.xffts_publish_flag()
    # d1 = d['dfs1'][0]
    # d2 = d['dfs2'][0]
    # d1_list.append(d1)
    # d2_list.append(d2)
    # tdim6_list.append([16384, 1, 1])
    # tmp_time = status.Time
    # tmp2 = datetime.fromtimestamp(tmp_time)
    # tmp3 = tmp2.strftime("%Y/%m/%d %H:%M:%S")
    # date_list.append(tmp3)
    # thot_list.append(temp)
    # vframe_list.append(dp1[0])
    # vframe2_list.append(dp1[0])
    # lst_list.append(status.LST)
    # az_list.append(status.Current_Az)
    # el_list.append(status.Current_El)
    # tau_list.append(tau)
    # hum_list.append(status.OutHumi)
    # tamb_list.append(status.OutTemp)
    # press_list.append(status.Press)
    # windspee_list.append(status.WindSp)
    # winddire_list.append(status.WindDir)
    # sobsmode_list.append('ON')
    # mjd_list.append(status.MJD)
    # secofday_list.append(status.Secofday)
    # subref_list.append(status.Current_M2)
    # tsys_list.append(tsys)
    # _2NDLO_list1.append(dp1[3]['sg21']*1000)
    # _2NDLO_list2.append(dp1[3]['sg22']*1000)
    # print("sg21 : ", dp1[3]['sg21']*1000)
    # print("sg22 : ", dp1[3]['sg22']*1000,"\n")
    log.info("stop")  # what is stoped?

    num += 1
    continue

print("R" + "\n")  # 最初と最後をhotではさむ
con.move_hot("in")
status = con.read_status()
while status.Current_Hot != "IN":
    print("wait hot_move...")
    status = con.read_status()
    time.sleep(0.5)
con.obs_status(active=True, current_num=num, current_position="HOT")

status = con.read_status()
temp = float(status.CabinTemp1)  # + 273.15
print("Temp: %.2f" % (temp))
print("get spectrum...")
# d = con.oneshot_achilles(exposure=integ_off)
con.xffts_publish_flag(scan_num=num, obs_mode="HOT")
time.sleep(integ_off)
con.xffts_publish_flag(scan_num=num, obs_mode="HOT")
# d1 = d['dfs1'][0]
# d2 = d['dfs2'][0]
# d1_list.append(d1)
# d2_list.append(d2)
# tdim6_list.append([16384, 1, 1])
# tmp_time = status.Time
# tmp2 = datetime.fromtimestamp(tmp_time)
# tmp3 = tmp2.strftime("%Y/%m/%d %H:%M:%S")
# date_list.append(tmp3)
# thot_list.append(temp)
# vframe_list.append(dp1[0])
# vframe2_list.append(dp1[0])
# lst_list.append(status.LST)
# az_list.append(status.Current_Az)
# el_list.append(status.Current_El)
# tau_list.append(tau)
# hum_list.append(status.OutHumi)
# tamb_list.append(status.OutTemp)
# press_list.append(status.Press)
# windspee_list.append(status.WindSp)
# winddire_list.append(status.WindDir)
# sobsmode_list.append('HOT')
# mjd_list.append(status.MJD)
# secofday_list.append(status.Secofday)
# subref_list.append(status.Current_M2)
# P_hot = numpy.sum(d1)
# tsys_list.append(0)
# _2NDLO_list1.append(dp1[3]['sg21']*1000)
# _2NDLO_list2.append(dp1[3]['sg22']*1000)
# print("sg21 : ", dp1[3]['sg21']*1000)
# print("sg22 : ", dp1[3]['sg22']*1000, "\n")

con.move_hot("out" + "\n")
log.info("observation end [observation time = {}]".format(time.time() - start_time))
con.move_stop()
con.dome_stop()


if obs["lo1st_sb_1"] == "U":
    ul = 1
else:
    ul = -1
imagfreq1 = obs["obsfreq_1"] - ul * obs["if1st_freq_1"] * 2
lofreq1 = obs["obsfreq_1"] - ul * obs["if1st_freq_1"] * 1

if obs["lo1st_sb_1"] == "U":
    ul1_1 = +1
else:
    ul1_1 = -1
if obs["lo2nd_sb_1"] == "U":
    ul1_2 = +1
else:
    ul1_2 = -1
if obs["lo3rd_sb_1"] == "U":
    ul1_3 = +1
else:
    ul1_3 = -1
ul1 = ul1_1 * ul1_2 * ul1_3
# print(ul1)
cdelt1_1 = (-1) * ul1 * 0.079370340319607024  # [(km/s)/ch]
# dv1 = (300000*cdelt1_1)/obs['restfreq_1']
crpix1_1 = (
    8191.5 - obs["vlsr"] / cdelt1_1 - (500 - obs["if3rd_freq_1"]) / 0.061038881767686015
)


if obs["lo1st_sb_2"] == "U":
    ul = 1
else:
    ul = -1
imagfreq2 = obs["obsfreq_2"] - ul * obs["if1st_freq_2"] * 2
lofreq2 = obs["obsfreq_2"] - ul * obs["if1st_freq_2"] * 1

if obs["lo1st_sb_2"] == "U":
    ul2_1 = +1
else:
    ul2_1 = -1
if obs["lo2nd_sb_2"] == "U":
    ul2_2 = +1
else:
    ul2_2 = -1
if obs["lo3rd_sb_2"] == "U":
    ul2_3 = +1
else:
    ul2_3 = -1
ul2 = ul2_1 * ul2_2 * ul2_3
# print(ul2)
cdelt1_2 = (-1) * ul2 * 0.0830267951512371  # [(km/s)/ch]
# dv2 = (300000*cdelt2)/obs['restfreq_2']
crpix1_2 = (
    8191.5 - obs["vlsr"] / cdelt1_2 - (500 - obs["if3rd_freq_2"]) / 0.061038881767686015
)

# # d1list
# read1 = {
#     "OBJECT": obs['object'],
#     "BANDWID": 1000000000,  # デバイスファイルに追加
#     "DATE-OBS": date_list,
#     "EXPOSURE": obs['exposure'],
#     "TSYS": tsys_list,
#     "DATA": d1_list,
#     "TDIM6": tdim6_list,  # デバイスファイルに追加
#     "TUNIT6": 'counts',  # デバイスファイルに追加
#     "CTYPE1": 'km/s',  # デバイスファイルに追加
#     "CRVAL1": 0,  # デバイスファイルに追加
#     "CRPIX1": crpix1_1,  # デバイスファイルに追加
#     "CDELT1": cdelt1_1,  # デバイスファイルに追加
#     "CTYPE2": 'deg',  # 未使用
#     "CRVAL2": 0,  # 未使用
#     "CTYPE3": 'deg',  # 未使用
#     "CRVAL3": 0,  # 未使用
#     "T_VLSR": 0,  # 未使用
#     "OBSERVER": obs['observer'],
#     "SCAN": 1,  # 要確認
#     "OBSMODE": obs['obsmode'],
#     "MOLECULE": obs['molecule_1'],
#     "TRANSITI": obs['transiti_1'],
#     "TEMPSCAL": 'TA',  # 未使用
#     "FRONTEND": 'nagoyaRX',  # デバイスファイルに追加
#     "BACKEND": 'nagoyaDFS',  # デバイスファイルに追加
#     "THOT": thot_list,
#     "TCOLD": 0,  # tcold_list
#     "FREQRES": 0.06103515625,  # デバイスファイルに追加[MHz]
#     "TIMESYS": 'UTC',  # 要確認
#     "VELDEF": 'RADI-LSR',
#     "VFRAME": vframe_list,
#     "VFRAME2": vframe2_list,
#     "OBSFREQ": obs['restfreq_1'],  # restfreq_1
#     "IMAGFREQ": imagfreq1,  # 要計算
#     "LST": lst_list,
#     "AZIMUTH": az_list,
#     "ELEVATIO": el_list,
#     "TAU": tau_list,
#     "HUMIDITY": hum_list,
#     "TAMBIENT": tamb_list,
#     "PRESSURE": press_list,
#     "WINDSPEE": windspee_list,
#     "WINDDIRE": winddire_list,
#     "BEAMEFF": 1,  # 未使用
#     "RESTFREQ": obs['restfreq_1'],
#     "SIG": 'T',  # 未使用
#     "CAL": 'F',  # 未使用
#     "SOBSMODE": sobsmode_list,
#     "QUALITY": 1,  # 未使用
#     "AOSLEN": 0.04,  # 未使用
#     "LOFREQ": lofreq1,  # 要計算
#     "SYNTH": 8038.000000000,  # 要調査[MHz;IF1]2ndLO
#     "FREQSWAM": 0,  # 要調査
#     "COORDSYS": obs['coordsys'],
#     "COSYDEL": obs['cosydel'],
#     "LAMDEL": obs['lamdel'],
#     "BETDEL": obs['betdel'],
#     "OTADEL": obs['otadel'],
#     "OTFVLAM": 0,
#     "OTFVBET": 0,
#     "OTFSCANN": 0,
#     "OTFLEN": 0,
#     "SUBSCAN": 0,  # 要実装
#     "MJD": mjd_list,
#     "SECOFDAY": secofday_list,
#     "SIDEBAND": obs['lo1st_sb_1'],
#     "_2NDSB": obs['lo2nd_sb_1'],
#     "_3RDSB": obs['lo3rd_sb_1'],
#     "_2NDLO": _2NDLO_list1,  # ドップラーシフト込み
#     "_3RDLO": obs['lo3rd_freq_1'],
#     "SUBREF": subref_list,
#     "LOCKSTAT": 'F'  # 未使用
# }

# # d2_list
# # d1list

# read2 = {
#     "OBJECT": obs['object'],
#     "BANDWID": 1000000000,  # デバイスファイルに追加
#     "EXPOSURE": obs['exposure'],
#     "DATE-OBS": date_list,
#     "TSYS": tsys_list,
#     "DATA": d2_list,
#     "TDIM6": tdim6_list,  # デバイスファイルに追加
#     "TUNIT6": 'counts',  # デバイスファイルに追加
#     "CTYPE1": 'km/s',  # デバイスファイルに追加
#     "CRVAL1": 0,  # デバイスファイルに追加
#     "CRPIX1": crpix1_2,  # デバイスファイルに追加
#     "CDELT1": cdelt1_2,  # デバイスファイルに追加
#     "CTYPE2": 'deg',  # 未使用
#     "CRVAL2": 0,  # 未使用
#     "CTYPE3": 'deg',  # 未使用
#     "CRVAL3": 0,  # 未使用
#     "T_VLSR": 0,  # 未使用
#     "OBSERVER": obs['observer'],
#     "SCAN": 1,  # 要確認
#     "OBSMODE": obs['obsmode'],
#     "MOLECULE": obs['molecule_2'],
#     "TRANSITI": obs['transiti_2'],
#     "TEMPSCAL": 'TA',  # 未使用
#     "FRONTEND": 'nagoyaRX',  # デバイスファイルに追加
#     "BACKEND": 'nagoyaDFS',  # デバイスファイルに追加
#     "THOT": thot_list,
#     "TCOLD": 0,  # tcold_list
#     "FREQRES": 0.06103515625,  # デバイスファイルに追加[MHz]
#     "TIMESYS": 'UTC',  # 要確認
#     "VELDEF": 'RADI-LSR',
#     "VFRAME": vframe_list,
#     "VFRAME2": vframe2_list,
#     "OBSFREQ": obs['restfreq_2'],
#     "IMAGFREQ": imagfreq2,  # 要計算
#     "LST": lst_list,
#     "AZIMUTH": az_list,
#     "ELEVATIO": el_list,
#     "TAU": tau_list,
#     "HUMIDITY": hum_list,
#     "TAMBIENT": tamb_list,
#     "PRESSURE": press_list,
#     "WINDSPEE": windspee_list,
#     "WINDDIRE": winddire_list,
#     "BEAMEFF": 1,  # 未使用
#     "RESTFREQ": obs['restfreq_2'],
#     "SIG": 'T',  # 未使用
#     "CAL": 'F',  # 未使用
#     "SOBSMODE": sobsmode_list,
#     "QUALITY": 1,  # 未使用
#     "AOSLEN": 0.04,  # 未使用
#     "LOFREQ": lofreq2,  # 要計算
#     "SYNTH": 9301.318999999,  # 要調査[MHz;IF2]2ndLO
#     "FREQSWAM": 0,  # 要調査
#     "COORDSYS": obs['coordsys'],
#     "COSYDEL": obs['cosydel'],
#     "LAMDEL": obs['lamdel'],
#     "BETDEL": obs['betdel'],
#     "OTADEL": obs['otadel'],
#     "OTFVLAM": 0,
#     "OTFVBET": 0,
#     "OTFSCANN": 0,
#     "OTFLEN": 0,
#     "SUBSCAN": 0,  # 要実装
#     "MJD": mjd_list,
#     "SECOFDAY": secofday_list,
#     "SIDEBAND": obs['lo1st_sb_2'],
#     "_2NDSB": obs['lo2nd_sb_2'],
#     "_3RDSB": obs['lo3rd_sb_2'],
#     "_2NDLO": _2NDLO_list2,  # ドップラーシフト込み
#     "_3RDLO": obs['lo3rd_freq_2'],
#     "SUBREF": subref_list,
#     "LOCKSTAT": 'F'  # 未使用
# }


# f1 = os.path.join(savedir, 'n2ps_%s_IF1.fits' % (timestamp))
# f2 = os.path.join(savedir, 'n2ps_%s_IF2.fits' % (timestamp))
# # numpy.save(f1+".npy",read1)
# # numpy.save(f2+".npy",read2)


# sys.path.append("/home/amigos/ros/src/necst/lib")
# n2fits_write.write(read1, f1)
# n2fits_write.write(read2, f2)

# timestamp = time.strftime('%Y%m%d_%H%M%S')
# dirname = timestamp
### Logging Start
con.pub_loggerflag("")
con.pub_analyexec(savedir, "ps")
con.obs_status(active=False)
con.pub_encdb_flag(False, os.path.join(savedir, "enc.db"))
