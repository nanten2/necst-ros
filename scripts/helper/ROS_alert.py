#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String

class alert(object):

    state = ''
    wind_speed = ''
    rain = ''
    memb = ''
    
    
    def __init__(self):
        self.pub = rospy.Publisher("alert", String, queue_size=10, latch=True) 
        self.sub = rospy.Subscriber("status_weather", Status_weather_msg, self.callback_weather)
        self.sub = rospy.Subscriber("status_dome", Status_dome_msg, self.callback_dome)

    def callback_weather(self, req):
        """ callback : status_weather """
        self.wind_speed = req.wind_sp
        self.rain = req.rain
        
    def callbck_dome(self, req):
        """ callback : status_dome """
        status_box = req.status
        self.memb = status_box[6]
        self.dome_r = status_box[2]
        self.dome_l = status_box[4]
        

    def alert(self):
        """ publish : alert """
        msg = String()
        while rospy.is_shutdown():
            msg.msg = self.state
            self.pub.publish(msg)
        return
    
    def thread_start(self):
        """ checking alert """
        self.stop_thread = threading.Event()
        self.thread = threading.Thread(target=self.check_thread,name="alert_check_thread")
        self.thread.start()
        
    def check_thread(self):
        """ checking alert """
        if float(self.wind_speed) > 10:
            self.state += "Warning : wind_speed over 10."
            rospy.logwarn(self.state)
        elif float(self.wind_speed) > 15:
            self.state += "Emergency : wind_speed over 15."
            rospy.logfatal(self.state)
        else:
            pass
        if 
            

        
