#!/usr/bin/env python3

import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import time
import threading
import astropy.units as u
from astropy.coordinates import SkyCoord, EarthLocation, AltAz, get_body
from astropy.time import Time
from datetime import datetime as dt
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from necst.msg import Status_weather_msg
from necst.msg import Status_dome_msg
from necst.msg import Dome_msg


class alert(object):

    state = ''
    wind_speed = 0
    rain = 0
    out_humi = 0
    memb = ''
    dome_r = ''
    dome_l = ''

    nanten2 = ""
    az_list = ""
    el_list = ""
    
    def __init__(self):
        self.pub_alert = rospy.Publisher("alert", String, queue_size=10, latch=True)
        #self.pub_emergency = rospy.Publisher('emergency_stop', Bool, queue_size = 10, latch = True)
        self.pub_stop = rospy.Publisher("move_stop", String, queue_size = 1, latch = True)
        self.pub_dome = rospy.Publisher('dome_move', Dome_msg, queue_size = 10, latch = True)
        self.sub = rospy.Subscriber("status_weather", Status_weather_msg, self.callback_weather)
        self.sub = rospy.Subscriber("status_dome", Status_dome_msg, self.callback_dome)
        self.nanten2 = EarthLocation(lat = -22.96995611*u.deg, lon = -67.70308139*u.deg, height = 4863.85*u.m)

        self.args = sys.argv
        self.args.append("")
        

    def callback_weather(self, req):
        """ callback : status_weather """
        self.wind_speed = req.wind_sp
        self.rain = req.rain
        self.out_humi = req.out_humi
        
    def callback_dome(self, req):
        """ callback : status_dome """
        #status_box = req.status
        self.memb = req.memb_pos#status_box[6]
        self.dome_r = req.right_pos#status_box[2]
        self.dome_l = req.left_pos#status_box[4]
        
    def callback_azel(self, req):
        self.az_list = req.az_list/3600.
        self.el_list = req.el_list/3600.
        pass

    def check_sun_position(self):
        now = dt.utcnow()
        sun = get_body("sun", Time(now))
        sun.location = self.nanten2
        azel = sun.altaz
        sun_az = azel.az.deg
        if sun_az > 240.:
            sun_az -= 360.
        sun_el = azel.alt.deg
        stop_flag = False
        az_list = self.az_list
        el_list = self.el_list
        for i in range(len(az_list)):
            if sun_az+15 > +240.:
                if (sun_az-360. + 15 >= az_list[i] >= -120.) or (sun_az-15 <= az_list[i] <= +240.):
                    stop_flag = True
                else:
                    pass
            elif sun_az-15 < -240.:
                if (sun_az+360. - 15 <= az_list[i] <= +120.) or (sun_az+15 >= az_list[i] >= -240.):
                    stop_flag = True
                else:
                    pass
            elif sun_az-15 <= az_list[i] <= sun_az+15:
                stop_flag = True
            else:
                pass
            if sun_el -15 <= el_list[i] <= sun_el + 15:
                stop_flag = True
        return stop_flag
        
        
    def alert(self):
        """ publish : alert """
        msg = String()
        while not rospy.is_shutdown():
            if self.state:
                msg.data = self.state
                self.pub_alert.publish(msg)
            time.sleep(0.3)
        return
    
    def thread_start(self):
        """ checking alert """
        self.stop_thread = threading.Event()
        self.thread = threading.Thread(target=self.check_thread,name="alert_check_thread")
        self.thread.start()
        
    def check_thread(self):
        """ checking alert 
        warning state --> only alert message
        emergency state --> [alert message] and [move stop] and [dome/membrane close]
        
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
        
        print("alert check start !!\n")
        while not rospy.is_shutdown():
            #print(self.state, self.wind_speed, self.rain, self.out_humi, self.memb, self.dome_r, self.dome_l)
            emergency = warning = ""

            if self.rain > 1.:
                self.state = "Warning : probably rainning...\n"
                warning =  self.state
            else:
                pass
            if self.out_humi > 60:
                self.state = "Warning : out_humi over 60 % \n"
                warning = self.state
            elif self.out_humi > 80:
                self.state = "Emergency : out_humi over 80 % \n"
                emergency = self.state
            else:
                pass            
            if 10. < self.wind_speed < 15.:
                self.state = "Warning : wind_speed over 10.\n"
                warning = self.state
            elif self.wind_speed > 15:
                self.state = "Emergency : wind_speed over 15.\n"
                emergency = self.state

            if self.args[1] == "radio":
                pass
            else:
                sun_pos = self.check_sun_position()
                if sun_pos:
                    self.state = "Emergency : antenna position near sun!! \n"
                    emergency = self.state
                
            if emergency:
                rospy.logfatal(emergency)
                self.pub_stop.publish(data = "stop")#antenna
                if self.memb.lower() == 'open':
                    self.pub_dome.publish(name='command', value='memb_close')
                if self.dome_r.lower() == 'open' or self.dome_l.lower() == 'open':
                    self.pub_dome.publish(name="command", value='dome_close')
                emergency = ""
            elif warning:
                rospy.logwarn(warning)
                warning = ""
            else:
                if self.state:
                    self.state = "error release !!\n\n\n"
                    rospy.loginfo(self.state)
                    self.state = ""
                    time.sleep(0.5)
                    self.pub_alert.publish(self.state)
                pass
            time.sleep(0.1)
        return

if __name__ == "__main__":
    rospy.init_node("alert")
    al = alert()
    al.thread_start()
    al.alert()
