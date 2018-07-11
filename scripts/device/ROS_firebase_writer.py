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
from necst.msg import Status_otf_msg

import signal
def handler(signal, frame):
    print("systen down...")
    sub1.unregister()
    sub2.unregister()
    sub3.unregister()
    time.sleep(1.)
    rospy.on_shutdown(_end)
    return
def _end():
    print("rospy shutdown...")
    return
signal.signal(signal.SIGINT, handler)

from firebase import firebase
fb = firebase.FirebaseApplication("https://test-d187a.firebaseio.com",None)
auth = firebase.FirebaseAuthentication("DgHtyfC5d1qcezGOBOsvrIOMRwdG9dG9fQ8xNVBz", "nascofirebase@gmail.com", extra={"id":123})
fb.authentication = auth

td={}
weather_status ={}
weather_status24 ={"humi24":[0]*24, "wind24":[0]*24}
device_status = {}
enc_status = {}
time_status = {}
otf_status = {}

no_alive = ""
launch = ""

def launch_check():
    global launch
    print("Please restart launch...")
    while not rospy.is_shutdown():
        names = rosnode.get_node_names()
        if "/check_alert" in names:
            break
        else:
            time.sleep(1.0)            
            pass
    f = open("/home/amigos/ros/src/necst/simulator/launch/simulator.launch","r")
    _line = f.readlines()
    f.close()
    del _line[0]
    del _line[-1]
    launch = [ast.literal_eval(i.split()[2].split("=")[1]) for i in _line]
    return

def node_check():
    global launch
    global no_alive
    old_alive = ""
    while not rospy.is_shutdown():
        names = rosnode.get_node_names()
        no_alive = [ i for i in launch if not "/"+str(i) in names]
        if no_alive == []:
            no_alive = ""
            old_alive = ""
        elif no_alive != old_alive:
            #rospy.logfatal(no_alive)
            ky = list(td.keys())
            for i in ky:
                if i in no_alive:
                    td[i] = ""
                    print("#########delete#######")
            old_alive = no_alive
        else:
            pass
                
        td["NodeStatus"] = no_alive        
    return 

def _topic(req):
    td[req.from_node] = req.active
    return

def _auth(req):
    device_status["Authority2"] = req.data
    return

def _dometrack(req):
    device_status["Dome_Track"] = req.data
    return

def topic():
    a=[]
    while not rospy.is_shutdown():
        #no_alive = node_check()
        #date = dt.utcnow()
        #timestamp = date.strftime("%Y-%m-%d %H:%M:%S")
        #ndata = [[timestamp, str([str(td), str(no_alive), str(weather_status), str(enc_status)])]]

        try:
            st = time.time()
            fb.put("", "/NECST/Monitor/Telescope/Node_status",td)
            ct = time.time()-st
            a.append(ct)
            print("time", ct)
            print("average", numpy.average(a))
        except Exception as e:
            rospy.logerr(e)
        time.sleep(1.0)
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
        #date = dt.utcnow()
        #timestamp = date.strftime("%Y-%m-%d %H:%M:%S")
        try:
            fb.put("", "/NECST/Monitor/Telescope/Encoder",enc_status)
            pass
        except Exception as e:
            rospy.logerr(e)
        time.sleep(0.5)
    return

def _otf(req):
    otf_status["active"] = req.active
    otf_status["target"] = req.target
    otf_status["xline"] = req.xline
    otf_status["yline"] = req.yline
    otf_status["xgrid"] = req.xgrid
    otf_status["ygrid"] = req.ygrid 
    otf_status["exposure_hot"] = req.exposure_hot
    otf_status["exposure_off"] = req.exposure_off
    otf_status["exposure_on"] = req.exposure_on
    otf_status["scan_direction"] = req.scan_direction
    otf_status["current_line"] = req.current_line
    otf_status["current_position"] = req.current_position
    otf_status["timestamp"] = req.timestamp
    return

def otf():
    while not rospy.is_shutdown():
        if not otf_status:
            time.sleep(0.5)
            continue
        try:
            fb.put("", "/NECST/Monitor/Telescope/Otf_status",otf_status)
            pass
        except Exception as e:
            rospy.logerr(e)
        time.sleep(0.5)
    return
    
if __name__ =="__main__":
    rospy.init_node("firebase_writer")
    sub1 = rospy.Subscriber("web_topic", Status_node_msg, _topic, queue_size=1)
    sub2 = rospy.Subscriber("read_status", Read_status_msg, _weather, queue_size=1)
    sub3 = rospy.Subscriber("status_encoder", Status_encoder_msg, _encoder, queue_size=1)
    sub4 = rospy.Subscriber("authority_check", String_necst, _auth, queue_size=1)
    sub5 = rospy.Subscriber("dome_track_flag", Bool_necst, _dometrack, queue_size=1)
    sub6 = rospy.Subscriber("timer", Status_timer_msg, _timer, queue_size=1)
    sub7 = rospy.Subscriber("otf_status", Status_otf_msg, _otf, queue_size=1)    
    
    
    nth = threading.Thread(target=node_check)
    nth.start()
    launch_check()
    tth = threading.Thread(target=topic)
    tth.start()
    wth = threading.Thread(target=weather)
    wth.start()
    dth = threading.Thread(target=device)
    dth.start()    
    eth = threading.Thread(target=encoder)
    eth.start()
    timeth = threading.Thread(target=timer)
    timeth.start()
    otfth = threading.Thread(target=otf)
    otfth.start()    
    rospy.spin()
