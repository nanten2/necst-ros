#!/usr/bin/env python3

import time
from datetime import datetime as dt
import sys

import rospy
from necst.msg import Status_weather_msg
from necst.msg import Status_obs_msg
from necst.msg import String_list_msg
from necst.msg import String_necst

import signal
def handler(signal, frame):
    sys.exit()
    print("***program end***")
    return
signal.signal(signal.SIGINT, handler)

node_name = "obs_log"

obs = ''
start_flag = False
observation = ""
weather = ""
hosei = ""
stop = ""
alert = ""

def _obs(req):
    global obs
    global start_flag
    obs = req
    start_flag = True
    return

def _weather(req):
    global weather
    weather = req
    return

def _hosei(req):
    global hosei
    hosei = req
    return

def _obs_stop(req):
    global stop
    stop = req.data
    return

def _alert(req):
    global alert
    alert = req.data
    return

def start():
    global stop
    stop = ""
    print("start")
    ctime = dt.utcnow()    
    filename = ctime.strftime("%Y%m%d")
    f = open("/home/amigos/data/obs_log/"+filename+".txt", "a")
    f.write("### Observation : " + obs.target + "\n")
    f.write("- UTC " + str(ctime.fromtimestamp(obs.timestamp)))
    f.write(": start observation\n")    
    f.write("- obsscript : " + obs.obs_script+"\n")
    f.write("- obsfile : " + obs.obs_file+"\n")
    f.write("- weather_status"+"\n")
    f.write("```\n")
    f.write(str(weather)+"\n")
    f.write("```\n")
    f.close()
    
    return

def moving():
    global hosei
    ctime = dt.utcnow()    
    filename = ctime.strftime("%Y%m%d")
    f = open("/home/amigos/data/obs_log/"+filename+".txt", "a")
    f.write("@time" + str(ctime.fromtimestamp(obs.timestamp))+"\n")
    f.write("@current_number : " + str(obs.current_num)+"\n")
    f.write("@current_position : " + str(obs.current_position)+"\n")
    f.write("@hosei_parameter : ")
    f.write(str(hosei.data[0])+"\n")
    if alert:
        f.write("- alert message"+"\n")
        f.write("-- "+str(alert)+"\n")
    f.close()
    return

def end():
    print("end")
    ctime = dt.utcnow()    
    filename = ctime.strftime("%Y%m%d")
    f = open("/home/amigos/data/obs_log/"+filename+".txt", "a")
    f.write("- UTC " + str(ctime.fromtimestamp(obs.timestamp)))
    f.write(" : end observation"+"\n")
    if stop:
        f.write("- limit error"+"\n")
        f.write("-- "+str(stop)+"\n")
    else:
        pass
    if alert:
        f.write("- alert message"+"\n")
        f.write("-- "+str(alert)+"\n")            
    f.write("- hosei_parameter"+"\n")
    f.write("```\n")
    f.write(str(hosei.data[0])+"\n")
    f.write("```\n")
    for i in range(1,len(hosei.data)):
        f.write("    @"+str(hosei.data[i])+"\n")    
    f.write("\n")
    f.close()
    return

def obs_record():
    observation = False
    while not rospy.is_shutdown() and not start_flag:
        time.sleep(1.)
    while not rospy.is_shutdown():
        if obs.active == True and observation == False:
            start()
            observation = True
            obs.active = ""
            pass
        elif obs.active == False and observation == True:
            end()
            observation = False
            obs.active = ""
        elif obs.active == False and observation == False:
            pass
        elif obs.active == True and observation == True:
            moving()
            obs.active = ""
        else:
            pass


if __name__ == "__main__":
    rospy.init_node(node_name)
    rospy.Subscriber("obs_status", Status_obs_msg, _obs)
    rospy.Subscriber("status_weather", Status_weather_msg, _weather)
    rospy.Subscriber("hosei_parameter", String_list_msg, _hosei)
    rospy.Subscriber("obs_stop", String_necst, _obs_stop)
    rospy.Subscriber("alert", String_necst, _alert, queue_size=1)
    obs_record()
