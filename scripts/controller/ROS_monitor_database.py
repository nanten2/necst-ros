#!/usr/bin/env python3

# =====
# this program is used by site and nagoya
# pick up data from firebase, and publish data
# =====


import os
import sys
import time
import threading
from datetime import datetime as dt

from firebase import firebase
fb = firebase.FirebaseApplication("https://test-d187a.firebaseio.com",None )
auth = firebase.FirebaseAuthentication("DgHtyfC5d1qcezGOBOsvrIOMRwdG9dG9fQ8xNVBz", "nascofirebase@gmail.com", extra={"id":123})
#fb = firebase.FirebaseApplication("https://nasco-obs-monitor.firebaseio.com",None)
#auth = firebase.FirebaseAuthentication("C8XaDyuGBjvjUBO5mnncYFrLWja9hEFYhdPsx5ow", "nascofirebase@gmail.com", extra={"id":123})
fb.authentication = auth

import rospy
from necst.msg import Bool_necst
from necst.msg import String_necst
from necst.msg import Read_status_msg
from necst.msg import Status_node_msg
from necst.msg import Status_timer_msg
from necst.msg import Status_obs_msg
from necst.msg import Float64_list_msg


# =======
# handler
# =======
import signal
def handler(signal, frame):
    rospy.on_shutdown(end)
    time.sleep(1.)
    return
signal.signal(signal.SIGINT, handler)

def end():
    print("*****Program is stop*****")
    return

# ====
# main
# ====

node_name = "monitor_datebase"

class server(object):

    data = ""
    wdata = ""

    def __init__(self):
        self.pub_auth = rospy.Publisher("web_auth", String_necst, queue_size=1)
        self.pub_status = rospy.Publisher("web_status", Read_status_msg, queue_size=1)
        self.pub_dome_track = rospy.Publisher("web_dome_track", Bool_necst, queue_size=1)
        self.pub_node = rospy.Publisher("web_node", Status_node_msg, queue_size=1)
        self.pub_timer = rospy.Publisher("web_timer", Status_timer_msg, queue_size=1)
        self.pub_alert = rospy.Publisher("web_alert", String_necst, queue_size=1)
        self.pub_obs = rospy.Publisher("web_obs", Status_obs_msg, queue_size=1)
        self.pub_next_obs = rospy.Publisher("web_next_obs", String_necst, queue_size=1)
        self.pub_humi = rospy.Publisher("web_humi24", Float64_list_msg, queue_size=1)
        self.pub_wind = rospy.Publisher("web_wind24", Float64_list_msg, queue_size=1)
        
        return
    
    def thread_start(self):
        self.initialize()
        th_read1 = threading.Thread(target=self.data_read_telescope)
        th_read1.start()
        th_read2 = threading.Thread(target=self.data_read_weather)
        th_read2.start()        
        th_pub = threading.Thread(target=self.pub_func)
        th_pub.start()
        return
        
    def initialize(self):
        print("first data loading...")
        self.data = fb.get("/NECST/Monitor/Telescope",None)
        self.wdata = fb.get("/NECST/Monitor/Instrument",None)
        time.sleep(2.)
        print("loading end.")         
        return
    
    def data_read_telescope(self):
        while not rospy.is_shutdown():
            try:
                self.data = fb.get("/NECST/Monitor/Telescope",None)
            except Exception as e:
                print(e)
            time.sleep(1.)
        return

    def data_read_weather(self):
        while not rospy.is_shutdown():
            try:
                self.wdata = fb.get("/NECST/Monitor/Instrument",None)
            except Exception as e:
                print(e)
            time.sleep(2.)
        return    

    def pub_func(self):
        while not rospy.is_shutdown():
            ctime = time.time()
            print(ctime)            
            self.pub_auth.publish(self.data["Device"]["Authority"], node_name, ctime)
            msg = Read_status_msg()
            msg.Current_Az = self.data["Encoder"]["Current_Az"]
            msg.Current_El = self.data["Encoder"]["Current_El"]
            msg.Command_Az = self.data["Device"]["Command_Az"]
            msg.Command_El = self.data["Device"]["Command_El"]
            msg.Deviation_Az = self.data["Device"]["Deviation_Az"]
            msg.Deviation_El = self.data["Device"]["Deviation_El"]
            msg.Drive_ready_Az = self.data["Device"]["Drive_ready_Az"]
            msg.Drive_ready_El = self.data["Device"]["Drive_ready_El"]
            msg.Authority = self.data["Device"]["Authority"]
            msg.Current_Dome = self.data["Device"]["Current_Dome"]
            msg.Door_Dome = self.data["Device"]["Door_Dome"]
            msg.Door_Membrane = self.data["Device"]["Door_Membrane"]
            msg.Current_M4 = self.data["Device"]["Current_M4"]
            msg.Current_Hot = self.data["Device"]["Current_Hot"]
            msg.Current_M2 = self.data["Device"]["Current_M2"]
            msg.Year = self.wdata["Weather"]["Year"]
            msg.Month = self.wdata["Weather"]["Month"]
            msg.Day = self.wdata["Weather"]["Day"]            
            msg.Hour = self.wdata["Weather"]["Hour"]
            msg.Min = self.wdata["Weather"]["Min"]
            msg.Sec = self.wdata["Weather"]["Sec"]
            msg.InTemp = self.wdata["Weather"]["InTemp"]
            msg.OutTemp = self.wdata["Weather"]["OutTemp"]
            msg.InHumi = self.wdata["Weather"]["InHumi"]
            msg.OutHumi = self.wdata["Weather"]["OutHumi"]
            msg.WindDir = self.wdata["Weather"]["WindDir"]
            msg.WindSp = self.wdata["Weather"]["WindSp"]
            msg.Press = self.wdata["Weather"]["Press"]
            msg.Rain = self.wdata["Weather"]["Rain"]
            msg.CabinTemp1 = self.wdata["Weather"]["CabinTemp1"]
            msg.DomeTemp1 = self.wdata["Weather"]["DomeTemp1"]
            msg.GenTemp1 = self.wdata["Weather"]["GenTemp1"]
            msg.CabinTemp2 = self.wdata["Weather"]["CabinTemp2"]
            msg.DomeTemp2 = self.wdata["Weather"]["DomeTemp2"]
            msg.GenTemp2 = self.wdata["Weather"]["GenTemp2"]
            msg.from_node = node_name
            msg.timestamp = ctime
            self.pub_status.publish(msg)

            self.pub_dome_track.publish(self.data["Device"]["Dome_Track"],node_name, ctime)
            self.pub_alert.publish(self.data["Error"]["error"], node_name, ctime)
            self.pub_next_obs.publish(self.data["Obs_status"]["next_obs"], node_name, ctime)

            msg = Float64_list_msg()
            msg.timestamp = ctime
            msg.from_node = node_name
            msg.data = self.wdata["Weather24"]["humi24"]
            self.pub_humi.publish(msg)
            msg.data =  self.wdata["Weather24"]["wind24"]
            self.pub_wind.publish(msg)            
            
            msg = Status_node_msg()
            msg.timestamp = ctime
            try:
                for i in list(self.data["Node_status"].keys()):
                    msg.from_node = i
                    msg.active = self.data["Node_status"][i]
                    self.pub_node.publish(msg)
                    time.sleep(0.01)
            except Exception as e:
                print(e)

            msg = Status_timer_msg()
            msg.lst_h = self.data["Timer"]["lst_h"]
            msg.lst_m = self.data["Timer"]["lst_m"]
            msg.lst_s = self.data["Timer"]["lst_s"]
            msg.utc_Y = self.data["Timer"]["utc_Y"]
            msg.utc_M = self.data["Timer"]["utc_M"]
            msg.utc_D = self.data["Timer"]["utc_D"]
            msg.utc_h = self.data["Timer"]["utc_h"]
            msg.utc_m = self.data["Timer"]["utc_m"]
            msg.utc_s = self.data["Timer"]["utc_s"]                        
            msg.mjd = self.data["Timer"]["mjd"]
            msg.secofday = self.data["Timer"]["secofday"]
            msg.from_node = node_name
            msg.timestamp = self.data["Timer"]["unix"]
            self.pub_timer.publish(msg)

            msg = Status_obs_msg()
            msg.active = self.data["Obs_status"]["active"]
            msg.current_num = self.data["Obs_status"]["current_num"]
            msg.current_position = self.data["Obs_status"]["current_position"]
            msg.exposure_hot = self.data["Obs_status"]["exposure_hot"]
            msg.exposure_off = self.data["Obs_status"]["exposure_off"]
            msg.exposure_on = self.data["Obs_status"]["exposure_on"]
            msg.num_on = self.data["Obs_status"]["num_on"]
            msg.num_seq = self.data["Obs_status"]["num_seq"]
            msg.obsmode = self.data["Obs_status"]["obsmode"]
            msg.scan_direction = self.data["Obs_status"]["scan_direction"]
            msg.target = self.data["Obs_status"]["target"]
            msg.timestamp = self.data["Obs_status"]["timestamp"]
            msg.xgrid = self.data["Obs_status"]["xgrid"]
            msg.ygrid = self.data["Obs_status"]["ygrid"]
            self.pub_obs.publish(msg)

            
            time.sleep(1.)
        return

"""
def observation():
    while flag:
        _queue = controll["Queue"]
        time.sleep(1.)
        if _queue != "":
            print("queue observation : ", _queue["observation"])
            con.queue_observation(_queue["observation"])
            fb.put("", "/NECST/Controll/Telescope/Queue", "")
        else:
            pass
        time.sleep(1.)
        print(str(dt.utcnow()))
    return
"""

class writer(object):
    
    def __init__(self):
        self.sub = rospy.Subscriber("web_queue", Bool_necst, self._queue, queue_size=1)
        return

    def _queue(self, req):
        fb.put("", "/NECST/Controll/Telescope/Device/Queue", req.data)
        return
        

if __name__ == "__main__":
    rospy.init_node(node_name)
    ww = writer()
    ss = server()
    ss.thread_start()
    rospy.spin()
    
