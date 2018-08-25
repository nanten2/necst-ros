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

from firebase import firebase
#fb = firebase.FirebaseApplication("https://test-d187a.firebaseio.com",None)
#auth = firebase.FirebaseAuthentication("DgHtyfC5d1qcezGOBOsvrIOMRwdG9dG9fQ8xNVBz", "nascofirebase@gmail.com", extra={"id":123})
"""tmp"""
fb = firebase.FirebaseApplication("https://nasco-obs-monitor.firebaseio.com",None)
auth = firebase.FirebaseAuthentication("C8XaDyuGBjvjUBO5mnncYFrLWja9hEFYhdPsx5ow", "nascofirebase@gmail.com", extra={"id":123})
fb.authentication = auth

import signal
def handler(signal, frame):
    print("systen down...")
    rospy.on_shutdown(_end)
    time.sleep(1.)    
    return
def _end():
    print("rospy shutdown...")
    return
signal.signal(signal.SIGINT, handler)


node_name = "firebase_writer"    

class writer(object):

    node_status={}
    launch_status = {}
    weather_status ={}
    weather_status24 ={"humi24":[0]*24, "wind24":[0]*24}
    device_status = {}
    
    time_status = {}
    obs_status = {}
    onepoint_status = {}
    
    enc_status = {}
    error_status = {"error":"no data"}
    
    # =============
    # main function
    # =============

    def __init__(self):
        sub1 = rospy.Subscriber("check_launch", String_list_msg, self._launch, queue_size=1)
        sub2 = rospy.Subscriber("check_node", Status_node_msg, self._node, queue_size=1)            
        sub3 = rospy.Subscriber("read_status", Read_status_msg, self._weather, queue_size=1)
        sub4 = rospy.Subscriber("status_encoder", Status_encoder_msg, self._encoder, queue_size=1)
        sub5 = rospy.Subscriber("authority_check", String_necst, self._auth, queue_size=1)
        sub6 = rospy.Subscriber("dome_track_flag", Bool_necst, self._dometrack, queue_size=1)
        sub7 = rospy.Subscriber("timer", Status_timer_msg, self._timer, queue_size=1)
        sub8 = rospy.Subscriber("obs_status", Status_obs_msg, self._obs, queue_size=1)
        sub9 = rospy.Subscriber("one_status", Status_onepoint_msg, self._onepoint, queue_size=1)    
        sub10 = rospy.Subscriber("next_obs", String_necst, self._nextobs, queue_size=1)
        sub11 = rospy.Subscriber("alert", String_necst, self._error, queue_size=1)
        self.start_thread()
        return

    def start_thread(self):
        nth = threading.Thread(target=self.node)
        nth.start()
        lth = threading.Thread(target=self.launch)
        lth.start()    
        wth = threading.Thread(target=self.weather)
        wth.start()
        dth = threading.Thread(target=self.device)
        dth.start()    
        eth = threading.Thread(target=self.encoder)
        eth.start()
        timeth = threading.Thread(target=self.timer)
        timeth.start()
        obsth = threading.Thread(target=self.obs)
        obsth.start()
        oneth = threading.Thread(target=self.onepoint)
        oneth.start()
        errth = threading.Thread(target=self.error)
        errth.start()
        print("loading end")
        return
        
    
    def _node(self, req):
        self.node_status[req.from_node] = req.active
        return
    
    def node(self):
        a=[]
        while not rospy.is_shutdown():
            try:
                st = time.time()
                fb.put("", "/NECST/Monitor/Telescope/Node_status",self.node_status)
                ct = time.time()-st
                a.append(ct)
                print("time", ct)
                print("average", numpy.average(a))
            except Exception as e:
                rospy.logerr(e)
            time.sleep(1.)
        return

    def _launch(self,req):
        self.launch_status["launch"] = req.data
        return

    def launch(self,):
        while not rospy.is_shutdown():
            try:
                fb.put("", "/NECST/Monitor/Telescope/Launch_status",self.launch_status)
            except Exception as e:
                rospy.logerr(e)
            time.sleep(1.)
        return

    def _auth(self,req):
        self.device_status["Authority2"] = req.data
        return

    def _dometrack(self,req):
        self.device_status["Dome_Track"] = req.data
        return

    def _weather(self,req):
        self.weather_status["Year"] = req.Year
        self.weather_status["Month"] = req.Month
        self.weather_status["Day"] = req.Day
        self.weather_status["Hour"] = req.Hour
        self.weather_status["Min"] = req.Min
        self.weather_status["Sec"] = req.Sec
        self.weather_status["InTemp"] = req.InTemp
        self.weather_status["OutTemp"] = req.OutTemp
        self.weather_status["InHumi"] = req.InHumi
        self.weather_status["OutHumi"] = req.OutHumi
        self.weather_status["WindSp"] = req.WindSp
        self.weather_status["WindDir"] = req.WindDir
        self.weather_status["Press"] = req.Press
        self.weather_status["Rain"] = req.Rain
        self.weather_status["CabinTemp1"] = req.CabinTemp1
        self.weather_status["CabinTemp2"] = req.CabinTemp2
        self.weather_status["DomeTemp1"] = req.DomeTemp1
        self.weather_status["DomeTemp2"] = req.DomeTemp2
        self.weather_status["GenTemp1"] = req.GenTemp1
        self.weather_status["GenTemp2"] = req.GenTemp2
        self.weather_status["Secofday"] = req.Secofday
    
        self.device_status["Authority"] = req.Authority
        self.device_status["Current_M2"] = req.Current_M2
        self.device_status["Current_M4"] = req.Current_M4
        self.device_status["Current_Hot"] = req.Current_Hot
        self.device_status["Drive_ready_Az"] = req.Drive_ready_Az
        self.device_status["Drive_ready_El"] = req.Drive_ready_El
        self.device_status["Current_Dome"] = req.Current_Dome
        self.device_status["Door_Dome"] = req.Door_Dome
        self.device_status["Door_Membrane"] = req.Door_Membrane
        self.device_status["Command_Az"] = req.Command_Az
        self.device_status["Command_El"] = req.Command_El   
        self.device_status["Deviation_Az"] = req.Deviation_Az
        self.device_status["Deviation_El"] = req.Deviation_El
        time.sleep(1.)
        return

    def weather(self):
        current = ""
        while not rospy.is_shutdown():
            if not self.weather_status:
                time.sleep(0.5)
                continue
            date = dt.utcnow()
            hh = date.hour
            if hh != current:
                self.weather_status24["humi24"][hh] = self.weather_status["InHumi"]
                self.weather_status24["wind24"][hh] = self.weather_status["WindSp"]
                current = hh
            else:
                pass
        
            try:
                fb.put("", "/NECST/Monitor/Instrument/Weather",self.weather_status)
                pass
            except Exception as e:
                rospy.logerr(e)
            try:
                fb.put("", "/NECST/Monitor/Instrument/Weather24",self.weather_status24)
                pass
            except Exception as e:
                rospy.logerr(e)
            
            time.sleep(1.0)
        return
    
    def device(self):
        while not rospy.is_shutdown():
            if not self.device_status:
                time.sleep(0.5)
                continue
            try:
                fb.put("", "/NECST/Monitor/Telescope/Device",self.device_status)
                pass
            except Exception as e:
                rospy.logerr(e)
            time.sleep(0.5)
        return

    def _timer(self,req):
        self.time_status["secofday"] = req.secofday
        self.time_status["lst_h"] = req.lst_h
        self.time_status["lst_m"] = req.lst_m
        self.time_status["lst_s"] = req.lst_s
        self.time_status["utc_Y"] = req.utc_Y
        self.time_status["utc_M"] = req.utc_M
        self.time_status["utc_D"] = req.utc_D
        self.time_status["utc_h"] = req.utc_h
        self.time_status["utc_m"] = req.utc_m
        self.time_status["utc_s"] = req.utc_s
        self.time_status["mjd"] = req.mjd
        self.time_status["unix"] = req.timestamp  
        time.sleep(0.1)
        return

    def timer(self,):
        while not rospy.is_shutdown():
            if not self.time_status:
                time.sleep(0.5)
                continue
            try:
                fb.put("", "/NECST/Monitor/Telescope/Timer",self.time_status)
                pass
            except Exception as e:
                rospy.logerr(e)
            time.sleep(0.5)
        return

    def _encoder(self,req):
        self.enc_status["Current_Az"] = req.enc_az
        self.enc_status["Current_El"] = req.enc_el
        return

    def encoder(self,):
        while not rospy.is_shutdown():
            if not self.enc_status:
                time.sleep(0.5)
                continue
            try:
                fb.put("", "/NECST/Monitor/Telescope/Encoder",self.enc_status)
                pass
            except Exception as e:
                rospy.logerr(e)
            time.sleep(0.5)
        return

    def _obs(self,req):
        self.obs_status["active"] = req.active
        self.obs_status["target"] = req.target
        self.obs_status["obsmode"] = req.obsmode    
        self.obs_status["num_on"] = req.num_on
        self.obs_status["num_seq"] = req.num_seq
        self.obs_status["xgrid"] = req.xgrid
        self.obs_status["ygrid"] = req.ygrid 
        self.obs_status["exposure_hot"] = req.exposure_hot
        self.obs_status["exposure_off"] = req.exposure_off
        self.obs_status["exposure_on"] = req.exposure_on
        self.obs_status["scan_direction"] = req.scan_direction
        self.obs_status["current_num"] = req.current_num
        self.obs_status["current_position"] = req.current_position
        self.obs_status["timestamp"] = req.timestamp
        self.obs_status["next_obs"] = "observation now"    
        return

    def _nextobs(self,req):
        self.obs_status["next_obs"] = req.data
        return

    def obs(self):
        while not rospy.is_shutdown():
            if not self.obs_status:
                time.sleep(0.5)
                continue
            try:
                fb.put("", "/NECST/Monitor/Telescope/Obs_status",self.obs_status)
                self.obs_status = {}
                pass
            except Exception as e:
                rospy.logerr(e)
            time.sleep(0.5)
        return

    def _onepoint(self,req):
        self.onepoint_status["one_active"] = req.active
        self.onepoint_status["one_target"] = req.target
        self.onepoint_status["one_num_on"] = req.num_on
        self.onepoint_status["one_num_seq"] = req.num_seq    
        self.onepoint_status["one_exposure_hot"] = req.exposure_hot
        self.onepoint_status["one_exposure_off"] = req.exposure_off
        self.onepoint_status["one_exposure_on"] = req.exposure_on
        self.onepoint_status["one_current_num"] = req.current_num
        self.onepoint_status["one_current_position"] = req.current_position
        self.onepoint_status["timestamp"] = req.timestamp
        self.onepoint_status["next_obs"] = "observation now"    
        return

    def onepoint(self,):
        self.onepoint_status = {}
        while not rospy.is_shutdown():
            if not self.onepoint_status:
                time.sleep(0.5)
                continue
            try:
                fb.put("", "/NECST/Monitor/Telescope/Onepoint_status",self.onepoint_status)
                self.onepoint_status = {}
                pass
            except Exception as e:
                rospy.logerr(e)
            time.sleep(0.5)
        return

    def _error(self,req):
        self.error_status["error"] = req.data
        return

    def error(self,):
        fb.put("", "/NECST/Monitor/Telescope/Error",self.error_status)    
        while not rospy.is_shutdown():
            if self.error_status["error"] == "no data":
                time.sleep(0.5)
                continue
            try:
                fb.put("", "/NECST/Monitor/Telescope/Error",self.error_status)
                self.error_status["error"] = "no data"
                pass
            except Exception as e:
                rospy.logerr(e)
            time.sleep(0.5)
        return

class server(object):

    def __init__(self):
        return

    def start_observation(self):
        while not rospy.is_shutdown():
            data = fb.get("/NECST/Controll/Telescope/Device/Queue",None)
            if not "":
                con.queue_observation(data)
                fb.put("", "/NECST/Controll/Telescope/Device/Queue", "")
            else:
                pass
            time.sleep(2.)
        return

if __name__ =="__main__":
    import ROS_controller
    con = ROS_controller.controller(node_name)
    ww = writer()
    ss = server()
    #ss.start_observation()
    rospy.spin()
