#!/usr/bin/env python3

from subprocess import Popen
import time
import threading
from datetime import datetime as dt
from datetime import time as dtime
import sys
import rospy
from necst.msg import Bool_necst
from necst.msg import String_necst

from firebase import firebase
fb = firebase.FirebaseApplication("https://necst-queue-list.firebaseio.com",None)
auth = firebase.FirebaseAuthentication("qxJcuaLUkeXULbJ0IYfNkDFBUMbPWgn6ZM1sme6W","nascofirebase@gmail.com", extra={"id":123})
fb.authentication = auth
 

current_num = 0
current_day = ""
obs_flag = False

node_name = "queue_obs"

import signal
def handler(signal, frame):
    global obs_flag
    obs_flag = False
    try:
        proc.send_signal(signal.SIGINT)
    except:
        pass
    print("**** system is down... ****")
    sys.exit()
signal.signal(signal.SIGINT, handler)

def _observation(req):
    global obs_flag
    global current_num
    obs_flag = req.data
    if obs_flag == False:
        try:
            proc.send_signal(signal.SIGINT)
        except:
            pass
        pub.publish("stop queue", node_name, time.time())        
        current_num = 0
    time.sleep(1.)
    return

def _obs_stop(req):
    global obs_flag
    global current_num
    #obs_flag = False
    try:
        proc.send_signal(signal.SIGINT)
    except:
        pass
    #current_num = 0
    return

def queue_check():
    global current_num
    global current_day
    ct = dt.utcnow()
    date = ct.strftime("%Y%m%d")
    if current_day != date:
        current_num = 0
        current_day = date
    else:
        pass
    try:
        print("loading file...")
        
        data = fb.get("/"+date+"/",None)
        print("finish loading")
    except:
        #if no file
        return "", ""
    if not data:
        print("no data")
        return "", ""
    length = len(list(data.keys())[0].split(":"))
    keys = [i+":00"*(3-length) for i in list(data.keys())]    
    keys = [dt.strptime(date + " "+i, "%Y%m%d %H:%M:%S") for i in keys]
    keys.sort()
    #print("queue list :", "\n", keys)
    for i in range(current_num, len(keys)):        
        ctime = dt.utcnow()
        if keys[i]>ctime:
            current_num = i
            while keys[i]>ctime and obs_flag==True:
                ctime = dt.utcnow()
                print("current time : ", ctime.strftime("%H:%M:%S"))
                print("next observation is : ", keys[i].strftime("%H:%M:%S"))
                pub.publish(str(keys[i]), node_name, time.time())
                print("\n")
                time.sleep(1.)
            if obs_flag == False:
                break
            _key = keys[i].strftime("%H:%M:%S")
            _key = _key.rsplit(":",(3-length))[0]
            try:
                ret =  list(data[_key].items())[0]
            except:
                try:
                    _key = _key.split("0",1)[1]
                    ret =  list(data[_key].items())[0]
                except Exception as e:
                    print(e)
            print(ret[0], ret[1])
            return ret[0], ret[1]
        else:
            continue

    return "", ""
        

rospy.init_node(node_name)
sub = rospy.Subscriber("queue_obs",Bool_necst, _observation, queue_size=1)
sub2 = rospy.Subscriber("obs_stop",String_necst, _obs_stop, queue_size=1)
pub = rospy.Publisher("next_obs", String_necst, queue_size=1)

while True:
    print("start")
    if not obs_flag:
        print("no reservation...\n")
        pub.publish("no reservation...", node_name, time.time())        
    while not obs_flag:
        time.sleep(1.)
    script, filename = queue_check()
    
    if not script:
        #obs_flag = False
        print("no queue_list")
        time.sleep(10.)
    else:
        if not str(filename):
            print("start observation : ", str(script))            
            if len(script.split("-")) == 3:
                _list = str(script).split(" ")
                cmd = "python "+_list[0]+".py"+" "+_list[1]+" "+_list[2]
            else:
                cmd = "python "+str(script)+".py"
        else:
            print("start observation : ", filename)
            cmd = "python "+str(script)+".py"+" --obsfile " + str(filename)
        cmd = cmd.split()
        proc = Popen(cmd)
        proc.wait()
    print("end\n")


    

