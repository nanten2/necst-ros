#!/usr/bin/env python3

import sys
import time
import threading

import rospy
from necst.msg import Status_weather_msg
from necst.msg import Alert_msg

node_name = "alert_encoder"

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
        self.pub_emergency = rospy.Publisher("alert_emergency", Alert_msg, queue_size=10, latch=True)
        self.pub_warning = rospy.Publisher("alert_warning", Alert_msg, queue_size=10, latch=True)        
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
        msg = Alert_msg()
        msg.from_node = node_name
        while not rospy.is_shutdown():
            msg.antenna_stop = self.antenna_stop
            msg.dome_stop = self.dome_stop
            msg.dome_close = self.dome_close
            msg.memb_close = self.memb_close
            msg.timestamp = time.time()
            if self.error_flag:
                msg.data = self.emergency
                self.pub_emergency.publish(msg)
                msg.data = self.warning
                self.pub_warning.publish(msg)
                if not self.emergency and not self.warning:
                    self.error_flag = False
                else:
                    pass
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
                self.antenna_stop = True
                self.dome_stop = True
                self.dome_close = True
                self.memb_close = True
                self.error_flag = True
            elif warning:
                self.warning = warning
                self.antenna_stop = False                
                self.dome_stop = False
                self.dome_close = False
                self.memb_close =False
                self.error_flag = True
                pass
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
