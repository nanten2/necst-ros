#!/usr/bin/env python3

import sys
import time
import threading
import rospy
from necst.msg import Status_weather_msg

node_name = "alert_weather"

class alert(object):

    wind_speed = ""
    rain = ""
    out_humi = ""
    
    emergency = ""
    warning = ""

    error_flag = False
    
    def __init__(self):
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
        error_flag = False
        while not rospy.is_shutdown():
            if self.emergency:
                rospy.logfatal(self.emergency)
                con.move_stop()
                con.dome_stop()
                con.memb_close()
                con.dome_close()
                con.alert(self.emergency, emergency=True)
                error_flag = True                
            elif self.warning:
                rospy.logwarn(self.warning)
                con.alert(self.warning)
                error_flag = True
            else:
                if error_flag:
                    msg = "weather error release !!\n\n\n"
                    rospy.loginfo(msg)
                    con.alert(msg)                    
                    time.sleep(0.5)
                    msg = ""
                    rospy.loginfo(msg)
                    con.alert(msg)                    
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
    import ROS_controller
    con = ROS_controller.controller(escape=node_name)
    al = alert()
    al.thread_start()
    rospy.spin()
