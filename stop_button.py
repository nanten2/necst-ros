#!/usr/bin/env python3

import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import tkinter
import rospy
from std_msgs.msg import String

import time
import os
import sys
sys.path.append("/home/necst/ros/src/necst/scripts/controller")
sys.path.append("/home/necst/ros/src/necst/lib")
    
root = tkinter.Tk()
root.title(u"STOP BUTTON")
root.geometry("400x50")

def stop_call(event):
    pub.publish()

#label
label = tkinter.Label(root, text=u'you can stop moving antenna by pressing this button')
label.pack()


#button
Button = tkinter.Button(text=u'stop', width=50)
Button.bind("<Button-1>",stop_call) 
Button.pack()

rospy.init_node('stop_telescope',anonymous = True)
pub = rospy.Publisher('move_stop', String, queue_size = 10,latch = True)
root.mainloop()
