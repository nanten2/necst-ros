#!/usr/bin/env python3

import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import tkinter
import rospy
from necst.msg import Bool_necst

import time
import os
import sys
sys.path.append("/home/necst/ros/src/necst/scripts/controller")
sys.path.append("/home/necst/ros/src/necst/lib")
    
root = tkinter.Tk()
root.title(u"STOP BUTTON")
root.geometry("400x50")

def stop_call(event):
    pub.publish(True,'stop_button',time.time())

#label
label = tkinter.Label(root, text=u'STOP ANTENNA')
label.pack()


#button
Button = tkinter.Button(text=u'stop', width=50)
Button.bind("<Button-1>",stop_call) 
Button.pack()

rospy.init_node('stop_telescope',anonymous = True)
pub = rospy.Publisher('move_stop', Bool_necst, queue_size = 1)
root.mainloop()
