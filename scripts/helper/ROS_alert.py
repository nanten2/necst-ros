#!/usr/bin/env python3

import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/necst/ros/src/necst/lib")
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
from necst.msg import Status_encoder_msg
from necst.msg import Status_antenna_msg
from necst.msg import list_azelmsg

class alert(object):

    state = ''
    wind_speed = 0
    rain = 0
    out_humi = 0
    memb = ''
    dome_r = ''
    dome_l = ''
    enc_az = ""
    enc_el = ""
    vel_az = ""
    vel_el = ""

    nanten2 = ""
    az_list = ""
    el_list = ""
    encoder_error = False
    error_flag = False
    alert_msg = ""
    
    def __init__(self):
        self.pub_alert = rospy.Publisher("alert", String, queue_size=10, latch=True)
        #self.pub_emergency = rospy.Publisher('emergency_stop', Bool, queue_size = 10, latch = True)
        self.pub_antenna = rospy.Publisher("move_stop", String, queue_size = 1, latch = True)
        self.pub_dome = rospy.Publisher('dome_move', Dome_msg, queue_size = 10, latch = True)
        self.sub = rospy.Subscriber("status_weather", Status_weather_msg, self.callback_weather)
        self.sub = rospy.Subscriber("status_dome", Status_dome_msg, self.callback_dome)
        self.sub = rospy.Subscriber("status_antenna", Status_antenna_msg, self.callback_command)
        self.sub = rospy.Subscriber("status_encoder", Status_encoder_msg, self.callback_encoder)
        self.sub = rospy.Subscriber("list_azel", list_azelmsg, self.callback_azel)
        self.nanten2 = EarthLocation(lat = -22.96995611*u.deg, lon = -67.70308139*u.deg, height = 4863.85*u.m)

        self.args = sys.argv
        self.args.append("")
        

    def callback_weather(self, req):
        """ callback : status_weather """
        self.wind_speed = req.wind_sp
        self.rain = req.rain
        self.out_humi = req.out_humi
        return
        
    def callback_dome(self, req):
        """ callback : status_dome """
        #status_box = req.status
        self.memb = req.memb_pos#status_box[6]
        self.dome_r = req.right_pos#status_box[2]
        self.dome_l = req.left_pos#status_box[4]
        return
        
    def callback_azel(self,req):
        """callback : azel list"""
        self.az_list = req.az_list
        self.el_list = req.el_list
        return
        
    def callback_command(self, req):
        """ callback : command azel """
        self.vel_az = req.command_azspeed
        self.vel_el = req.command_elspeed
        return

    def callback_encoder(self, req):
        """ callback : encoder_status """
        self.enc_az = req.enc_az
        self.enc_el = req.enc_el
        return

    def check_encoder_error(self):
        while not self.encoder_error:
            az_flag = el_flag = False
            if self.vel_az != 0:
                start_az = self.enc_az
                az_flag = True
            else:
                pass
            if self.vel_el != 0:
                start_el = self.enc_el
                el_flag = True
            else:
                pass
            time.sleep(10.)
            if az_flag:
                if self.enc_az == start_az:
                    self.encoder_error = True
                else:
                    az_flag = False
            if el_flag:
                if self.enc_el == start_el:
                    self.encoder_error =True
                else:
                    el_flag = False
        return self.encoder_error
            
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
                if (sun_az-360. + 15 >= az_list[i]/3600. >= -120.) or (sun_az-15 <= az_list[i]/3600. <= +240.):
                    stop_flag = True
                else:
                    pass
            elif sun_az-15 < -240.:
                if (sun_az+360. - 15 <= az_list[i]/3600. <= +120.) or (sun_az+15 >= az_list[i]/3600. >= -240.):
                    stop_flag = True
                else:
                    pass
            elif sun_az-15 <= az_list[i]/3600. <= sun_az+15:
                stop_flag = True
            else:
                pass
            if sun_el -15 <= el_list[i]/3600. <= sun_el + 15:
                stop_flag = True
        return stop_flag
        
    def alert(self):
        """ publish : alert """
        while not rospy.is_shutdown():
            self.pub_alert.publish(data=self.alert_msg)
            time.sleep(0.1)
        return
    
    def thread_start(self):
        """ checking alert """
        self.stop_thread = threading.Event()
        self.thread_encoder = threading.Thread(target=self.check_encoder_error)
        self.thread_encoder.setDaemon(True)
        self.thread_encoder.start()

        self.thread_alert = threading.Thread(target=self.alert)
        self.thread_alert.setDaemon(True)
        self.thread_alert.start()
        
        
    def check_alert(self):
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
        |sun_az - real_az| < 15 [deg]
        |sun_el - real_el| < 15 [deg]
        if encoder is not move
        """
        
        print("alert check start !!\n")
        self.alert_msg = ""
        error_flag = False
        while not rospy.is_shutdown():
            #print(self.state, self.wind_speed, self.rain, self.out_humi, self.memb, self.dome_r, self.dome_l)
            emergency = ""
            warning = ""
            
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

            if self.encoder_error:
                emergency += "Emergency : encoder can't count!! \n"
            else:
                pass
                    
            if self.args[1] == "radio":
                pass
            else:
                sun_pos = self.check_sun_position()
                if sun_pos:
                    emergency += "Emergency : antenna position near sun!! \n"
                
            if emergency:
                rospy.logfatal(emergency)
                self.pub_antenna.publish(data = "stop")#antenna
                self.pub_dome.publish(name='command', value='dome_stop')
                if self.memb.lower() == 'open':
                    self.pub_dome.publish(name='command', value='memb_close')
                if self.dome_r.lower() == 'open' or self.dome_l.lower() == 'open':
                    self.pub_dome.publish(name="command", value='dome_close')
                self.alert_msg = emergency
                error_flag = True
            elif warning:
                rospy.logwarn(warning)
                self.alert_msg = warning
                error_flag = True
            else:
                if error_flag:
                    self.alert_msg = "error release !!\n\n\n"
                    rospy.loginfo(self.alert_msg)
                    time.sleep(0.5)
                    self.alert_msg = ""
                    rospy.loginfo(self.alet_msg)
                    error_flag = False
                else:
                    pass
                pass
            time.sleep(0.1)
        return

if __name__ == "__main__":
    rospy.init_node("alert")
    al = alert()
    al.thread_start()
    al.check_alert()
