import os
import sys
import time
import threading
from datetime import datetime as dt
from firebase import firebase
fb = firebase.FirebaseApplication("https://test-d187a.firebaseio.com",None)

import signal
def handler(signal, frame):
    global flag
    flag = False
    time.sleep(1.)
    print("*****Program is stop*****")
    return
signal.signal(signal.SIGINT, handler)
flag = True

def server():
    global device
    initialize()
    print("server start!!!")
    while flag:
        try:
            device = fb.get("/NECST/Controll/Telescope/Device",None)
        except Exception as e:
            print(e)
        time.sleep(0.5)
    return

def initialize():
    #init_param = fb.get("/NECST/Monitor/Telescope/Device",None)
    #print("start initialize...\n")
    #time.sleep(3)
    fb.put("", "/NECST/Controll/Telescope",{"Device":{"authority":"", "dome":"", "memb":"", "drive":"", "emergency":"", "hot":"", "m4":"", "m2":"" },"Onepoint":"","Planet":""})
    
    return
    
def start_thread():
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
    #th8 = threading.Thread(target=m2)
    #th8.start()
    th9 = threading.Thread(target=hot)
    th9.start()
    th10 = threading.Thread(target=authority)
    th10.start()

    return

    

        
def onepoint():
    while flag:
        _one = fb.get("/NECST/Controll/Telescope/Onepoint",None)
        if _one != "":
            con.onepoint_move(_one["x"], _one["y"], _one["coord"], _one["off_x"], _one["off_y"], _one["offcoord"], _one["hosei"], _one["lamda"], _one["dcos"], _one["limit"])
            fb.put("", "/NECST/Controll/Telescope/Onepoint", "")
        else:
            pass
        time.sleep(0.5)
    return

def linear():
    return

def planet():
    while flag:
        _planet = fb.get("/NECST/Controll/Telescope/Planet",None)
        if _planet != "":
            con.planet_move(_planet["planet"], _planet["off_x"], _planet["off_y"], _planet["offcoord"], _planet["hosei"], _planet["lamda"], _planet["dcos"], _planet["limit"])
            fb.put("", "/NECST/Controll/Telescope/Planet", "")
        else:
            pass
        time.sleep(0.5)
    return

def observation():
    #os.system("python /home/amigos/ros/src/necst/obs_scripts/ROS_ps.py --obsfile ps_test.obs")
    #os.system("")
    return

def drive():
    while flag:
        if device["drive"] != "":
            con.drive(device["drive"])
            device["drive"] = ""
            fb.put("", "/NECST/Controll/Telescope/Device/drive", "")
        else:
            pass
        time.sleep(0.5)
    return

"""
def stop():
    _st = device["stop"]
    while flag:    
        if _st != device["stop"]:
            con.move_stop(device["stop"])
            _st = device["stop"]
        else:
            pass
    return
"""

def emergency():
    while flag:    
        if device["emergency"] != "":
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
        if device["dome"] != "":
            con.dome(device["dome"])
            fb.put("", "/NECST/Controll/Telescope/Device/dome", "")
        else:
            pass
        time.sleep(0.5)
    return

def memb():
    while flag:    
        if device["memb"] != "":
            con.memb(device["memb"])
            fb.put("", "/NECST/Controll/Telescope/Device/memb", "")
        else:
            pass    
        time.sleep(0.5)
    return

def m4():
    while flag:    
        if device["m4"] != "":
            con.move_m4(device["m4"])
            fb.put("", "/NECST/Controll/Telescope/Device/m4", "")
        else:
            pass
        time.sleep(0.5)
    return

def hot():
    while flag:    
        if device["hot"] != "":
            con.move_hot(device["hot"])
            fb.put("", "/NECST/Controll/Telescope/Device/hot", "")
        else:
            pass
        time.sleep(0.5)
    return 

"""
def m2():
    while flag:    
        if device["m2"] != "":
            con.move_m2(device["m2"])
            fb.put("", "/NECST/Controll/Telescope/Device/m2", "")
        else:
            pass    
        time.sleep(0.5)
    return
"""
"""
def ac240():
    dr = device["drive"]
    while flag:    
        if dr != device["drive"]:
            con.drive(device["drive"])
            dr = device["drive"]
        else:
            pass    
    return
"""
"""
def xffts():
    dr = device["drive"]
    while flag:    
        if dr != device["drive"]:
            con.drive(device["drive"])
            dr = device["drive"]
        else:
            pass    
    return
"""

def authority():
    while flag:    
        if device["authority"] != "":
            if device["authority"] == "get":
                con.get_authority()
            elif device["authority"] == "release":
                con.release_authority()
            fb.put("", "/NECST/Controll/Telescope/Device/authority", "")
        else:
            pass    
        time.sleep(0.5)
    return

if __name__ == "__main__":
    device = fb.get("/NECST/Controll/Telescope/Device",None)
    print(device)
    import ROS_controller
    con = ROS_controller.controller()
    start_thread()
    server()
