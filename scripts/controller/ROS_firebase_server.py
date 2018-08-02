#!/usr/bin/env python3

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

# =======
# handler
# =======
import signal
def handler(signal, frame):
    global flag
    flag = False
    fb.put("","/NECST/Controll/Telescope/Device/user","")    
    time.sleep(1.)
    print("*****Program is stop*****")
    return
signal.signal(signal.SIGINT, handler)

# =======
# default
# =======
flag = True

queue = ""
stop = ""

# ====
# main
# ====
def server():
    global queue
    global stop
    print("server start!!!")
    start_thread()
    fb.put("", "/NECST/Controll/Telescope/Queue", "")
    fb.put("", "/NECST/Controll/Telescope/Stop", "")            
    while flag:
        try:
            queue = fb.get("/NECST/Controll/Telescope/Queue",None)
            stop = fb.get("/NECST/Controll/Telescope/Stop",None)
        except Exception as e:
            print(e)
        time.sleep(2.)
    return

def start_thread():
    th1 = threading.Thread(target=observation)
    th1.start()
    th2 = threading.Thread(target=emergency)
    th2.start()    
    return
        
def observation():
    global queue
    while flag:
        if queue != "":
            print("queue observation : ", queue)
            con.queue_observation(queue)
            fb.put("", "/NECST/Controll/Telescope/Queue", "")
        else:
            pass
        time.sleep(1.)
        print(str(dt.utcnow()))
    return


def emergency():
    global stop
    while flag:    
        if stop != "":
            con.move_stop(stop)
            fb.put("", "/NECST/Controll/Telescope/Stop", "")            
        else:
            pass
    return


if __name__ == "__main__":
    import ROS_controller
    con = ROS_controller.controller()
    server()
