#!/usr/bin/env python3

import os
import sys
import time
import threading
from datetime import datetime as dt
from firebase import firebase
fb = firebase.FirebaseApplication("https://test-d187a.firebaseio.com",None)
auth = firebase.FirebaseAuthentication("DgHtyfC5d1qcezGOBOsvrIOMRwdG9dG9fQ8xNVBz", "nascofirebase@gmail.com", extra={"id":123})
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
controll = {"Device":{"user":"", "authority":"", "dome":"", "memb":"", "drive":"", "emergency":"", "hot":"", "m4":"", "m2":"" },"Onepoint":"","Planet":"", "Queue":""}

# ====
# main
# ====
def server():
    global controll
    print("server start!!!")
    initialize()
    start_thread()    
    while flag:
        try:
            controll = fb.get("/NECST/Controll/Telescope",None)
        except Exception as e:
            print(e)
        time.sleep(1.)
    return

def initialize():
    fb.put("", "/NECST/Controll/Telescope",{"Device":{"user":"", "authority":"", "dome":"", "memb":"", "drive":"", "emergency":"", "hot":"", "m4":"", "m2":"" },"Onepoint":"","Planet":"", "Queue":""})
    name = con.check_my_node()
    fb.put("","/NECST/Controll/Telescope/Device/user",name)    
    return
    
def start_thread():
    print("start program!!")
    initialize()
    th1 = threading.Thread(target=onepoint)
    th1.start()
    th2 = threading.Thread(target=planet)
    th2.start()
    th3 = threading.Thread(target=drive)
    th3.start()
    th4 = threading.Thread(target=emergency)
    th4.start()
    th5 = threading.Thread(target=dome)
    th5.start()
    th6 = threading.Thread(target=memb)
    th6.start()
    th7 = threading.Thread(target=m4)
    th7.start()
    th8 = threading.Thread(target=m2)
    th8.start()
    th9 = threading.Thread(target=hot)
    th9.start()
    th10 = threading.Thread(target=authority)
    th10.start()
    th11 = threading.Thread(target=observation)
    th11.start()
    return
        
def onepoint():
    while flag:
        _one = controll["Onepoint"]
        time.sleep(1.)
        if _one != "":
            print("onepoint_move")
            con.onepoint_move(_one["x"], _one["y"], _one["coord"], _one["off_x"], _one["off_y"], _one["offcoord"], _one["hosei"], _one["lamda"], _one["dcos"], _one["limit"])
            fb.put("", "/NECST/Controll/Telescope/Onepoint", "")
        else:
            pass
        time.sleep(1.)
    return

def linear():
    return

def planet():
    while flag:
        _planet = controll["Planet"]        
        time.sleep(1.)
        if _planet != "":
            print("planet_move : ", _planet)
            con.planet_move(_planet["planet"], _planet["off_x"], _planet["off_y"], _planet["offcoord"], _planet["hosei"], _planet["lamda"], _planet["dcos"], _planet["limit"])
            fb.put("", "/NECST/Controll/Telescope/Planet", "")
        else:
            pass
        time.sleep(1.)
    return

def observation():
    while flag:
        _queue = controll["Queue"]
        time.sleep(1.)
        if _queue != "":
            print("queue observation : ", _queue["observation"])
            con.release_authority()
            con.queue_observation(_queue["observation"])
            fb.put("", "/NECST/Controll/Telescope/Queue", "")
        else:
            pass
        time.sleep(1.)
        print(str(dt.utcnow()))
    return

def drive():
    while flag:
        if controll["Device"]["drive"] != "":
            print("drive : ", controll["Device"]["drive"])
            con.drive(controll["Device"]["drive"])
            controll["Device"]["drive"] = ""
            fb.put("", "/NECST/Controll/Telescope/Device/drive", "")
        else:
            pass
        time.sleep(1.)
    return

"""
def stop():
    _st = Device["stop"]
    while flag:    
        if _st != Device["stop"]:
            con.move_stop(Device["stop"])
            _st = Device["stop"]
        else:
            pass
    return
"""

def emergency():
    while flag:    
        if controll["Device"]["emergency"] != "":
            con.move_stop()
            con.dome_stop()
            fb.put("", "/NECST/Controll/Telescope/Device/emergency", "")
        else:
            pass
        time.sleep(0.5)
    return

def otf():
    return

def dome():
    while flag:
        if controll["Device"]["dome"] != "":
            print("dome : ", controll["Device"]["dome"])
            if controll["Device"]["dome"] == "tracking":
                con.dome_track()
            elif controll["Device"]["dome"] == "trackend":
                con.dome_track_end()
            else:
                con.dome(controll["Device"]["dome"])
            fb.put("", "/NECST/Controll/Telescope/Device/dome", "")
        else:
            pass
        time.sleep(1.)
    return

def memb():
    while flag:    
        if controll["Device"]["memb"] != "":
            print("memb : ", controll["Device"]["memb"])            
            con.memb(controll["Device"]["memb"])
            fb.put("", "/NECST/Controll/Telescope/Device/memb", "")
        else:
            pass    
        time.sleep(1.)
    return

def m4():
    while flag:    
        if controll["Device"]["m4"] != "":
            print("m4 : ", controll["Device"]["m4"])            
            con.move_m4(controll["Device"]["m4"])
            fb.put("", "/NECST/Controll/Telescope/Device/m4", "")
        else:
            pass
        time.sleep(1.)
    return

def hot():
    while flag:    
        if controll["Device"]["hot"] != "":
            print("hot : ", controll["Device"]["hot"])            
            con.move_hot(controll["Device"]["hot"])
            fb.put("", "/NECST/Controll/Telescope/Device/hot", "")
        else:
            pass
        time.sleep(1.)
    return 


def m2():
    while flag:    
        if controll["Device"]["m2"] != "":
            print("m2 : ", controll["Device"]["m2"])            
            con.move_m2(controll["Device"]["m2"])
            fb.put("", "/NECST/Controll/Telescope/Device/m2", "")
        else:
            pass    
        time.sleep(1.)
    return

"""
def ac240():
    dr = controll["Device"]["drive"]
    while flag:    
        if dr != controll["Device"]["drive"]:
            con.drive(controll["Device"]["drive"])
            dr = controll["Device"]["drive"]
        else:
            pass    
    return
"""
"""
def xffts():
    dr = controll["Device"]["drive"]
    while flag:    
        if dr != controll["Device"]["drive"]:
            con.drive(controll["Device"]["drive"])
            dr = controll["Device"]["drive"]
        else:
            pass    
    return
"""

def authority():
    while flag:    
        if controll["Device"]["authority"] != "":
            print("authority : ", controll["Device"]["authority"])            
            if controll["Device"]["authority"] == "get":
                con.get_authority()
            elif controll["Device"]["authority"] == "release":
                con.release_authority()
            fb.put("", "/NECST/Controll/Telescope/Device/authority", "")
        else:
            pass    
        time.sleep(0.5)
    return

if __name__ == "__main__":
    import ROS_controller
    con = ROS_controller.controller()
    server()
