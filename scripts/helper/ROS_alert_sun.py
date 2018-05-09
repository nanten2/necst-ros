#!/usr/bin/env python3

import sys
import time
import threading

import astropy.units as u
from astropy.coordinates import SkyCoord, EarthLocation, AltAz, get_body
from astropy.time import Time
from datetime import datetime as dt
from datetime import timedelta
import rospy

from necst.msg import Alert_msg
from necst.msg import List_coord_msg

node_name = "alert_sun"

class alert(object):

    memb = ''
    dome_r = ''
    dome_l = ''

    az_list = ""
    el_list = ""

    error_flag = False
    sun_limit = False
    alert_msg = ""
    emergency = ""
    warning = ""

    antenna_stop = False
    dome_stop = False
    dome_close = False
    memb_close = False
    
    def __init__(self):
        self.pub_emergency = rospy.Publisher("alert_emergency", Alert_msg, queue_size=10, latch=True)
        self.pub_warning = rospy.Publisher("alert_warning", Alert_msg, queue_size=10, latch=True)        
        self.sub = rospy.Subscriber("list_azel", List_coord_msg, self.callback_azel)
        self.nanten2 = EarthLocation(lat = -22.96995611*u.deg, lon = -67.70308139*u.deg, height = 4863.85*u.m)
        return
           
    def callback_azel(self,req):
        """callback : azel list"""
        self.az_list = req.x_list
        self.el_list = req.y_list
        return
    
    def thread_start(self):
        """ checking alert """
        self.thread_alert = threading.Thread(target=self.pub_status)
        self.thread_alert.setDaemon(True)
        self.thread_alert.start()
        self.thread_sun = threading.Thread(target=self.check_sun_position)
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

    def check_sun_position(self):
        """
        Emergency
        ---------
        |sun_az - real_az| < 15 [deg]
        |sun_el - real_el| < 15 [deg]

        """
        while not rospy.is_shutdown():
            warning = ""
            emergency = ""
            now = dt.utcnow()
            sun = get_body("sun", Time(now))
            sun.location = self.nanten2
            azel = sun.altaz
            sun_az = azel.az.arcsec
            sun_el = azel.alt.arcsec
            az_list = self.az_list
            el_list = self.el_list

            check_az = [i for i in az_list if 0<abs(i-sun_az)<15*3600. or 345*3600.<abs(i-sun_az)<360*3600.]
            check_el = [i for i in el_list if 0<abs(i-sun_el)<15*3600.]
            if check_az or check_el:
                emergency += "Emergency : antenna position near sun!! \n"
            else:
                pass
            if emergency:
                self.emergency = emergency
                self.dome_move = True
                self.dome_close = True
                self.memb_close = True
                self.error_flag = True                
            elif warning:
                self.warning = warning
                self.dome_move = False
                self.dome_close = False
                self.memb_close = False
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
