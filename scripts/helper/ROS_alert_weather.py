#!/usr/bin/env python3

import sys
import time
import threading

import rospy
from necst.msg import String_necst
from necst.msg import Bool_necst
from necst.msg import Dome_msg
from necst.msg import Status_antenna_msg
from necst.msg import Status_encoder_msg
from necst.msg import Status_weather_msg


node_name = "alert_weather"

class alert(object):

    wind_speed = ""
    rain = ""
    out_humi = ""
    
    emergency = ""
    warning = ""

    antenna_stop = False
    dome_stop = False
    dome_close = False
    memb_close = False

    error_flag = False
    
    def __init__(self):
        self.pub_alert = rospy.Publisher("alert", String_necst, queue_size=10, latch=True)        
        self.pub_antenna = rospy.Publisher("move_stop", Bool_necst, queue_size = 1, latch = True)
        self.pub_dome = rospy.Publisher('dome_move', Dome_msg, queue_size = 10, latch = True)        
        self.sub = rospy.Subscriber("status_weather", Status_weather_msg, self.callback_weather)
        
    def callback_weather(self, req):
        """ callback : status_weather """
        self.wind_speed = req.wind_sp
        self.rain = req.rain
        self.out_humi = req.out_humi
        return
    
    def thread_start(self):
        """ checking alert """
        self.thread_alert = threading.Thread(target=self.pub_status)
        self.thread_alert.setDaemon(True)
        self.thread_alert.start()
        self.thread_sun = threading.Thread(target=self.check_weather)
        self.thread_sun.setDaemon(True)
        self.thread_sun.start()
        return

    def pub_status(self):
        """ publish : alert """
        antenna = Bool_necst()
        antenna.data = True
        antenna.from_node = node_name
        dome = Dome_msg()
        dome.name = "command"
        dome.from_node = node_name
        msg = String_necst()
        msg.from_node = node_name
        error_flag = False
        while not rospy.is_shutdown():
            timestamp = time.time()
            if self.emergency:
                rospy.logfatal(self.emergency)
                antenna.timestamp = timestamp
                self.pub_antenna.publish(antenna)
                dome.value = "dome_stop"
                self.pub_dome.publish(dome)
                dome.value = "memb_close"
                self.pub_dome.publish(dome)
                dome.value = "dome_close"
                self.pub_dome.publish(dome)
                msg.data = self.emergency
                self.pub_alert.publish(msg)
                error_flag = True                
            elif self.warning:
                rospy.logwarn(self.warning)
                msg.data = self.warning
                self.pub_alert.publish(msg)
                error_flag = True
            else:
                if error_flag:
                    msg.data = "weather error release !!\n\n\n"
                    rospy.loginfo(msg.data)
                    self.pub_alert.publish(msg)                    
                    time.sleep(0.5)
                    msg.data = ""
                    rospy.loginfo(msg.data)
                    self.pub_alert.publish(msg)                    
                    error_flag = False
                else:
                    pass
            time.sleep(0.01)
        return

    def check_weather(self):
        """
        Warning
        -------
        rain > 1 [mm]
        out humi > 60 [%]
        wind_speed > 10 [km/s]

        Emergency
        ---------
        out humi > 80 [%]
        wind_speed > 15 [km/s]

        """
        while not self.out_humi:
            time.sleep(1.)
        while not rospy.is_shutdown():
            warning = ""
            emergency = ""
            if self.rain > 1.:
                warning += "Warning : probably rainning...\n"
            else:
                pass
            if 60 < self.out_humi <= 80:
                warning += "Warning : out_humi over 60 % \n"
            elif self.out_humi > 80:
                emergency += "Emergency : out_humi over 80 % \n"
            else:
                pass            
            if 10. < self.wind_speed < 15.:
                warning += "Warning : wind_speed over 10.\n"
            elif self.wind_speed >= 15:
                emergency += "Emergency : wind_speed over 15.\n"
            else:
                pass
            if emergency:
                self.emergency = emergency
            elif warning:
                self.warning = warning
                self.emergency = ""                
            else:
                self.emergency = ""
                self.warning = ""
                pass
            time.sleep(1.)
        return
    
if __name__ == "__main__":
    rospy.init_node(node_name)
    al = alert()
    al.thread_start()
    rospy.spin()
