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


node_name = "alert_ping"

class alert(object):

    ping_flag = False
    timestamp = 0
    
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
                    msg.data = "encoder error release !!\n\n\n"
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
    rospy.init_node(node_name)
    al = alert()
    al.thread_start()
    rospy.spin()
