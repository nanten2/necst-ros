#!/usr/bin/env python3

import sys
import time
import threading

import rospy
from necst.msg import Bool_necst

node_name = "alert_ping"

class alert(object):

    ping_flag = False
    timestamp = 0
    
    emergency = ""
    warning = ""

    error_flag = False
    
    def __init__(self):
        self.sub = rospy.Subscriber("status_ping", Bool_necst, self._ping)
        return

    def _ping(self, req):
        """ callback : encoder_status """
        self.ping_flag = req.data
        self.timestamp = req.timestamp
        return        
    
    def thread_start(self):
        """ checking alert """
        self.thread_alert = threading.Thread(target=self.pub_status)
        self.thread_alert.setDaemon(True)
        self.thread_alert.start()
        self.thread_sun = threading.Thread(target=self.check_ping)
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
                msg = self.emergency
                con.alert(msg)
                error_flag = True
            elif self.warning:
                rospy.logwarn(self.warning)
                msg = self.warning
                con.alert(msg)
                error_flag = True
            else:
                if error_flag:
                    msg = "encoder error release !!\n\n\n"
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

    def check_ping(self):
        """
        Warning
        -------
        
        Emergency
        ---------
        "ping NICT" is no connect for 10 [s]

        """
        while not rospy.is_shutdown():
            warning = ""
            emergency = ""
            ping_time = self.timestamp
            
            if self.ping_flag:
                if time.time() - ping_time > 10.:
                    emergency += "Emergency : ping to NICT is no connection\n"
                else:
                    pass
            else:
                # stop alert situation
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
