#!/usr/bin/env python3

import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import tkinter
import rospy
from std_msgs.msg import String

import time
import os
import sys
import argparse
sys.path.append("/home/necst/ros/src/necst/scripts/controller")
import ROS_controller
sys.path.append("/home/necst/ros/src/necst/lib")
import obs_log
ctrl = ROS_controller.controller()


# Info
# ----

name = 'initialize'
description = 'Initialize antenna'

# Default parameters
# ------------------

opt = ''

# Argument handler
# ================

p = argparse.ArgumentParser(description=description)
p.add_argument('--opt', type=str,
               help='For optical. Need 1.')

args = p.parse_args()

if args.opt is not None: opt = args.opt

# Main
# ====

def initialize(event):
    print(event)
    obs_log.start_script(name)
    obs_log.weather_log()
    
    time.sleep(2)
    ctrl.drive_on()
    ctrl.contactor_on()
    print("dome_open")
    ctrl.dome_open()

    if opt:
        print("memb_open")
        ctrl.memb_open()
        print("Init end")
    ctrl.dome_track()
    time.sleep(0.8)
    ctrl.dome_track_end()
        
    obs_log.end_script(name)
    
root = tkinter.Tk()
root.title(u"STOP BUTTON")
root.geometry("400x200")



#def DeleteEntryValue(event):
    #EditBox.delete(0, Tkinter.END)

def stop_call(event):
    pub.publish()

#label
label = tkinter.Label(root, text=u'you can stop antenna move by pressing this button')
label.pack()


#button
Button = tkinter.Button(text=u'stop', width=50)
Button.bind("<Button-1>",stop_call) 
Button.pack()

#button
Button2 = tkinter.Button(text=u'Initialize', width=50)
Button2.bind("<Button-1>",initialize) 
Button2.pack()

#rospy.init_node('stop_telescope',anonymous = True)
pub = rospy.Publisher('move_stop', String, queue_size = 10,latch = True)
root.mainloop()
