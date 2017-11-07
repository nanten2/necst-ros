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
        self.sub = rospy.Subscriber("status_weather", Status_weather_msg, self.callback_memb)

    def callback_weather(self, req):
        """ callback : status_weather """
        self.wind_speed = req.wind_sp
        self.rain = req.rain
        
    def callbck_memb(self, req):
        """ callback : status_memb """
        status_box = req.status
        self.memb = status_box[6]

    def alert(self):
        """ publish : alert """
        msg = String()
        while rospy.is_shutdown():
            msg.msg = self.state
            self.pub.publish(msg)
        return

    def thread_start(self):
        """ check_thread start """
        self.stop_thread = threading.Event()
        self.check_state = threading
    
    def check_thread(self):
        """ checking alert """
        self.stop_thread = threading.Event()
        self.thread = threading.Thread(target=self.check_thread,name="alert_check_thread")
        self.thread.start()
        
    def check_thread(self):
        """ checking alert """
        if float(self.wind_speed) > 10:
            self.state += "Warning : wind_speed over 10."
            rospy.logwarn(self.state)
        if
            

        
