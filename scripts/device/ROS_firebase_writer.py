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

td={}
weather_status ={}
device_status = {}
enc_status = {}

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
            rospy.logfatal(no_alive)
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
    device_status["Drive_Ready_Az"] = req.Drive_ready_Az
    device_status["Drive_Ready_El"] = req.Drive_ready_El
    device_status["Current_Dome"] = req.Current_Dome
    device_status["Door_Dome"] = req.Door_Dome
    device_status["Door_Membrane"] = req.Door_Membrane
    

    
    
    return

def weather():
    while not rospy.is_shutdown():
        if not weather_status:
            time.sleep(0.5)
            continue
        #date = dt.utcnow()
        #timestamp = date.strftime("%Y-%m-%d %H:%M:%S")
        #wdata = [[timestamp, str(weather_status)]]
        try:
            fb.put("", "/NECST/Monitor/Instrument/Weather",weather_status)
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

def _encoder(req):
    enc_status["Az"] = req.enc_az
    enc_status["El"] = req.enc_el
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

if __name__ =="__main__":
    rospy.init_node("node_status")
    sub1 = rospy.Subscriber("web_topic", Status_node_msg, _topic, queue_size=1)
    sub2 = rospy.Subscriber("read_status", Read_status_msg, _weather, queue_size=1)
    sub3 = rospy.Subscriber("status_encoder", Status_encoder_msg, _encoder, queue_size=1)      
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
    rospy.spin()
