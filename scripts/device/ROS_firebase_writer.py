#!/usr/bin/env python3

import numpy
import rospy
import rosnode
import ast
import time
import threading
from datetime import datetime as dt
import sys
sys.path.append("/home/amigos/git")
from necst.msg import Status_node_msg
from necst.msg import Read_status_msg
from necst.msg import Status_encoder_msg
from necst.msg import Status_timer_msg
from necst.msg import String_necst
from necst.msg import Bool_necst
from necst.msg import Status_obs_msg
from necst.msg import Status_onepoint_msg
from necst.msg import String_list_msg

# =======
# handler
# =======
import signal
def handler(signal, frame):
    print("systen down...")
    sub1.unregister()
    sub2.unregister()
    sub3.unregister()
    sub4.unregister()
    sub5.unregister()
    sub6.unregister()
    sub7.unregister()
    sub8.unregister()
    sub9.unregister()
    sub10.unregister()    
    time.sleep(1.)
    rospy.on_shutdown(_end)
    return
def _end():
    print("rospy shutdown...")
    return
signal.signal(signal.SIGINT, handler)

# ===================
# firebase initialize
# ===================
from firebase import firebase
#fb = firebase.FirebaseApplication("https://test-d187a.firebaseio.com",None)
#auth = firebase.FirebaseAuthentication("DgHtyfC5d1qcezGOBOsvrIOMRwdG9dG9fQ8xNVBz", "nascofirebase@gmail.com", extra={"id":123})
"""tmp"""
fb = firebase.FirebaseApplication("https://nasco-obs-monitor.firebaseio.com",None)
auth = firebase.FirebaseAuthentication("C8XaDyuGBjvjUBO5mnncYFrLWja9hEFYhdPsx5ow", "nascofirebase@gmail.com", extra={"id":123})

fb.authentication = auth

# =================
# default parameter
# =================
node_status={}
launch_status = {}
weather_status ={}
weather_status24 ={"humi24":[0]*24, "wind24":[0]*24}
device_status = {}

time_status = {}
obs_status = {}
onepoint_status = {}

node_name = "firebase_writer"
enc_status = {}
# =============
# main function
# =============
def _node(req):
    node_status[req.from_node] = req.active
    return

def node():
    a=[]
    while not rospy.is_shutdown():
        try:
            st = time.time()
            fb.put("", "/NECST/Monitor/Telescope/Node_status",node_status)
            ct = time.time()-st
            a.append(ct)
            print("time", ct)
            print("average", numpy.average(a))
        except Exception as e:
            rospy.logerr(e)
        time.sleep(1.)
    return

def _launch(req):
    launch_status["launch"] = req.data
    return

def launch():
    while not rospy.is_shutdown():
        try:
            fb.put("", "/NECST/Monitor/Telescope/Launch_status",launch_status)
        except Exception as e:
            rospy.logerr(e)
        time.sleep(1.)
    return

def _auth(req):
    device_status["Authority2"] = req.data
    return

def _dometrack(req):
    device_status["Dome_Track"] = req.data
    return

def _weather(req):
    weather_status["Year"] = req.Year
    weather_status["Month"] = req.Month
    weather_status["Day"] = req.Day
    weather_status["Hour"] = req.Hour
    weather_status["Min"] = req.Min
    weather_status["Sec"] = req.Sec
    weather_status["InTemp"] = req.InTemp
    weather_status["OutTemp"] = req.OutTemp
    weather_status["InHumi"] = req.InHumi
    weather_status["OutHumi"] = req.OutHumi
    weather_status["WindSp"] = req.WindSp
    weather_status["WindDir"] = req.WindDir
    weather_status["Press"] = req.Press
    weather_status["Rain"] = req.Rain
    weather_status["CabinTemp1"] = req.CabinTemp1
    weather_status["CabinTemp2"] = req.CabinTemp2
    weather_status["DomeTemp1"] = req.DomeTemp1
    weather_status["DomeTemp2"] = req.DomeTemp2
    weather_status["GenTemp1"] = req.GenTemp1
    weather_status["GenTemp2"] = req.GenTemp2
    weather_status["Secofday"] = req.Secofday

    device_status["Authority"] = req.Authority
    device_status["Current_M2"] = req.Current_M2
    device_status["Current_M4"] = req.Current_M4
    device_status["Current_Hot"] = req.Current_Hot
    device_status["Drive_ready_Az"] = req.Drive_ready_Az
    device_status["Drive_ready_El"] = req.Drive_ready_El
    device_status["Current_Dome"] = req.Current_Dome
    device_status["Door_Dome"] = req.Door_Dome
    device_status["Door_Membrane"] = req.Door_Membrane
    device_status["Command_Az"] = req.Command_Az
    device_status["Command_El"] = req.Command_El   
    device_status["Deviation_Az"] = req.Deviation_Az
    device_status["Deviation_El"] = req.Deviation_El
    time.sleep(1.)
    return

def weather():
    current = ""
    while not rospy.is_shutdown():
        if not weather_status:
            time.sleep(0.5)
            continue
        date = dt.utcnow()
        hh = date.hour
        if hh != current:
            weather_status24["humi24"][hh] = weather_status["InHumi"]
            weather_status24["wind24"][hh] = weather_status["WindSp"]
            current = hh
        else:
            pass
        
        try:
            fb.put("", "/NECST/Monitor/Instrument/Weather",weather_status)
            pass
        except Exception as e:
            rospy.logerr(e)
        try:
            fb.put("", "/NECST/Monitor/Instrument/Weather24",weather_status24)
            pass
        except Exception as e:
            rospy.logerr(e)
            
        time.sleep(1.0)
    return

def device():
    while not rospy.is_shutdown():
        if not device_status:
            time.sleep(0.5)
            continue
        try:
            fb.put("", "/NECST/Monitor/Telescope/Device",device_status)
            pass
        except Exception as e:
            rospy.logerr(e)
        time.sleep(0.5)
    return

def _timer(req):
    time_status["secofday"] = req.secofday
    time_status["lst_h"] = req.lst_h
    time_status["lst_m"] = req.lst_m
    time_status["lst_s"] = req.lst_s
    time_status["utc_Y"] = req.utc_Y
    time_status["utc_M"] = req.utc_M
    time_status["utc_D"] = req.utc_D
    time_status["utc_h"] = req.utc_h
    time_status["utc_m"] = req.utc_m
    time_status["utc_s"] = req.utc_s
    time_status["mjd"] = req.mjd
    time_status["unix"] = req.timestamp  
    time.sleep(0.1)
    return

def timer():
    while not rospy.is_shutdown():
        if not time_status:
            time.sleep(0.5)
            continue
        try:
            fb.put("", "/NECST/Monitor/Telescope/Timer",time_status)
            pass
        except Exception as e:
            rospy.logerr(e)
        time.sleep(0.5)
    return

def _encoder(req):
    enc_status["Current_Az"] = req.enc_az
    enc_status["Current_El"] = req.enc_el
    return

def encoder():
    while not rospy.is_shutdown():
        if not enc_status:
            time.sleep(0.5)
            continue
        try:
            fb.put("", "/NECST/Monitor/Telescope/Encoder",enc_status)
            pass
        except Exception as e:
            rospy.logerr(e)
        time.sleep(0.5)
    return

def _obs(req):
    obs_status["active"] = req.active
    obs_status["target"] = req.target
    obs_status["obsmode"] = req.obsmode    
    obs_status["num_on"] = req.num_on
    obs_status["num_seq"] = req.num_seq
    obs_status["xgrid"] = req.xgrid
    obs_status["ygrid"] = req.ygrid 
    obs_status["exposure_hot"] = req.exposure_hot
    obs_status["exposure_off"] = req.exposure_off
    obs_status["exposure_on"] = req.exposure_on
    obs_status["scan_direction"] = req.scan_direction
    obs_status["current_num"] = req.current_num
    obs_status["current_position"] = req.current_position
    obs_status["timestamp"] = req.timestamp
    obs_status["next_obs"] = "observation now"    
    return

def _nextobs(req):
    obs_status["next_obs"] = req.data
    return

def obs():
    global obs_status
    while not rospy.is_shutdown():
        if not obs_status:
            time.sleep(0.5)
            continue
        try:
            fb.put("", "/NECST/Monitor/Telescope/Obs_status",obs_status)
            obs_status = {}
            pass
        except Exception as e:
            rospy.logerr(e)
        time.sleep(0.5)
    return

def _onepoint(req):
    onepoint_status["one_active"] = req.active
    onepoint_status["one_target"] = req.target
    onepoint_status["one_num_on"] = req.num_on
    onepoint_status["one_num_seq"] = req.num_seq    
    onepoint_status["one_exposure_hot"] = req.exposure_hot
    onepoint_status["one_exposure_off"] = req.exposure_off
    onepoint_status["one_exposure_on"] = req.exposure_on
    onepoint_status["one_current_num"] = req.current_num
    onepoint_status["one_current_position"] = req.current_position
    onepoint_status["timestamp"] = req.timestamp
    onepoint_status["next_obs"] = "observation now"    
    return

def onepoint():
    onepoint_status = {}
    while not rospy.is_shutdown():
        if not onepoint_status:
            time.sleep(0.5)
            continue
        try:
            fb.put("", "/NECST/Monitor/Telescope/Onepoint_status",onepoint_status)
            onepoint_status = {}
            pass
        except Exception as e:
            rospy.logerr(e)
        time.sleep(0.5)
    return


if __name__ =="__main__":
    rospy.init_node(node_name)
    sub1 = rospy.Subscriber("check_launch", String_list_msg, _launch, queue_size=1)
    sub2 = rospy.Subscriber("check_node", Status_node_msg, _node, queue_size=1)            
    sub3 = rospy.Subscriber("read_status", Read_status_msg, _weather, queue_size=1)
    sub4 = rospy.Subscriber("status_encoder", Status_encoder_msg, _encoder, queue_size=1)
    sub5 = rospy.Subscriber("authority_check", String_necst, _auth, queue_size=1)
    sub6 = rospy.Subscriber("dome_track_flag", Bool_necst, _dometrack, queue_size=1)
    sub7 = rospy.Subscriber("timer", Status_timer_msg, _timer, queue_size=1)
    sub8 = rospy.Subscriber("obs_status", Status_obs_msg, _obs, queue_size=1)
    sub9 = rospy.Subscriber("one_status", Status_onepoint_msg, _onepoint, queue_size=1)    
    sub10 = rospy.Subscriber("next_obs", String_necst, _nextobs, queue_size=1)

    
    
    nth = threading.Thread(target=node)
    nth.start()
    lth = threading.Thread(target=launch)
    lth.start()    
    wth = threading.Thread(target=weather)
    wth.start()
    dth = threading.Thread(target=device)
    dth.start()    
    eth = threading.Thread(target=encoder)
    eth.start()
    timeth = threading.Thread(target=timer)
    timeth.start()
    obsth = threading.Thread(target=obs)
    obsth.start()
    oneth = threading.Thread(target=onepoint)
    oneth.start()        
    rospy.spin()
