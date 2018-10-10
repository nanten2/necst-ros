#!/usr/bin/env python3

import sys
import time
import threading
from datetime import datetime as dt
from datetime import timedelta
import rospy

from necst.msg import List_coord_msg

node_name = "alert_sun"

class alert(object):

    az_list = ""
    el_list = ""

    error_flag = False
    sun_limit = False
    alert_msg = ""
    emergency = ""
    warning = ""
    status_dome = "OPEN"
    status_memb = "OPEN"    

    def __init__(self):
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
        error_flag = False
        while not rospy.is_shutdown():
            if self.emergency:
                rospy.logfatal(self.emergency)
                con.move_stop()
                con.dome_stop()
                con.dome_close()
                con.memb_close()
                msg = self.emergency
                con.alert(msg,emergency=True)
                error_flag = True                
            elif self.warning:
                rospy.logwarn(self.warning)
                msg = self.warning
                con.alert(msg)
                error_flag = True
            else:
                if error_flag:
                    msg = "sun_position error release !!\n\n\n"
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

            #try:
            status = con.read_status()
            self.status_dome = status.Door_Dome.upper()
            self.status_memb = status.Door_Membrane.upper()
            #except Exception as e:
                #self.dome_flag = True
                #rospy.logerr(e)
            if check_az and check_el and self.status_dome != "CLOSE" and self.status_memb != "CLOSE":
                emergency += "Emergency : antenna position near sun!! \n"
            elif check_az and check_el:
                warning += "Warning : antenna position near sun!! \n"
                warning += "dome status : "
                warning += self.status_dome+"\n"
                warning += "memb status : "
                warning += self.status_memb+"\n"             
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
    import astropy.units as u
    from astropy.coordinates import SkyCoord, EarthLocation, AltAz, get_body
    from astropy.time import Time    
    al = alert()
    al.thread_start()
    rospy.spin()
