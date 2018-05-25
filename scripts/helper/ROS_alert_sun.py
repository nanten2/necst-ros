#!/usr/bin/env python3

import sys
import time
import threading

from datetime import datetime as dt
from datetime import timedelta
import rospy

from necst.msg import String_necst
from necst.msg import Bool_necst
from necst.msg import Dome_msg
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
        self.pub_alert = rospy.Publisher("alert", String_necst, queue_size=10, latch=True)        
        self.pub_antenna = rospy.Publisher("move_stop", Bool_necst, queue_size = 1, latch = True)
        self.pub_dome = rospy.Publisher('dome_move', Dome_msg, queue_size = 10, latch = True)                
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
        #antenna = Bool_necst()
        #antenna.data = True
        #antenna.from_node = node_name
        dome = Dome_msg()
        dome.name = "command"
        dome.from_node = node_name
        msg = String_necst()
        msg.from_node = node_name
        error_flag = False
        self.pub_alert.publish(msg)                
        while not rospy.is_shutdown():
            timestamp = time.time()
            if self.emergency:
                rospy.logfatal(self.emergency)
                #antenna.timestamp = timestamp
                #self.pub_antenna.publish(antenna)
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
                    msg.data = "sun_position error release !!\n\n\n"
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
            sun = get_body("sun", Time(now))#+timedelta(hours=12)))
            sun.location = self.nanten2
            azel = sun.altaz
            sun_az = azel.az.arcsec
            sun_el = azel.alt.arcsec
            az_list = self.az_list
            el_list = self.el_list

            check_az = [i for i in az_list if 0<abs(i-sun_az)<15*3600. or 345*3600.<abs(i-sun_az)<360*3600.]
            check_el = [i for i in el_list if 0<abs(i-sun_el)<15*3600.]
            if check_az and check_el:
                emergency += "Emergency : antenna position near sun!! \n"
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
    import astropy.units as u
    from astropy.coordinates import SkyCoord, EarthLocation, AltAz, get_body
    from astropy.time import Time    
    al = alert()
    al.thread_start()
    rospy.spin()
