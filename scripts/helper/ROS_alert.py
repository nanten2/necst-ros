#!/usr/bin/env python3

import sys
import time
import threading

import rospy
from necst.msg import String_necst
from necst.msg import Bool_necst
from necst.msg import Dome_msg
from necst.msg import Alert_msg
from necst.msg import Status_dome_msg


node_name = "alert"

class alert(object):

    memb = ''
    dome_r = ''
    dome_l = ''

    error_flag = False
    sun_limit = False
    alert_msg = ""
    emergency = ""
    warning = ""

    antenna_stop = ""
    dome_stop = ""
    dome_close = ""
    memb_close = ""
    
    def __init__(self):
        self.pub_alert = rospy.Publisher("alert", String_necst, queue_size=10, latch=True)
        self.pub_antenna_stop = rospy.Publisher("move_stop", Bool_necst, queue_size = 1, latch = True)
        self.pub_dome = rospy.Publisher('dome_move', Dome_msg, queue_size = 10, latch = True)
        self.sub = rospy.Subscriber("status_dome", Status_dome_msg, self.callback_dome)        
        self.sub = rospy.Subscriber("alert_warning", Alert_msg, self.call_warning, queue_size=1)
        self.sub = rospy.Subscriber("alert_emergency", Alert_msg, self.call_emergency, queue_size=1)
        self.start_time = time.time()
        return

    def callback_dome(self, req):
        """ callback : status_dome """
        self.memb = req.memb_pos
        self.dome_r = req.right_pos
        self.dome_l = req.left_pos
        return
    
    def call_warning(self, req):
        if self.start_time > req.timestamp:
            pass
        else:
            self.warning = req.data
            self.antenna_stop = req.antenna_stop
            self.dome_stop = req.dome_stop
            self.dome_close = req.dome_close
            self.memb_close = req.memb_close
            self.error_flag = True 
            return

    def call_emergency(self, req):
        if self.start_time > req.timestamp:
            pass
        else:
            self.emergency = req.data                        
            self.antenna_stop = req.antenna_stop
            self.dome_stop = req.dome_stop
            self.dome_close = req.dome_close
            self.memb_close = req.memb_close
            self.error_flag = True
            return
            
    def thread_start(self):
        """ checking alert """
        self.thread_alert = threading.Thread(target=self.alert)
        self.thread_alert.setDaemon(True)
        self.thread_alert.start()
        return

    def alert(self):
        """ publish : alert """
        msg = String_necst()
        msg.from_node = node_name
        while not rospy.is_shutdown():
            msg.data = self.alert_msg
            msg.timestamp = time.time()
            self.pub_alert.publish(msg)
            time.sleep(0.01)
        return

    def check_alert(self):
        """ checking alert 
        warning state --> only alert message
        emergency state --> [alert message] and [move stop] and [dome/membrane close]
        """
        print("alert check start !!\n")
        while not rospy.is_shutdown():
            if self.emergency:
                timestamp = time.time()
                rospy.logfatal(self.emergency)
                if self.antenna_stop:
                    self.pub_antenna_stop.publish(data=True, from_node=node_name, timestamp=timestamp)
                msg = Dome_msg()
                msg.name = "command"
                msg.from_node = node_name
                msg.timestamp = time.time()

                if self.dome_stop:
                    msg.value = "dome_stop"
                    self.pub_dome.publish(msg)
                if self.memb_close:
                    if self.memb.lower() == 'open':
                        msg.value = "memb_close"                    
                        self.pub_dome.publish(msg)
                if self.dome_close:
                    if self.dome_r.lower() == 'open' or self.dome_l.lower() == 'open':
                        msg.value = "dome_close"
                        self.pub_dome.publish(msg)
                self.alert_msg = self.emergency
                self.error_flag = True
            elif self.warning:
                rospy.logwarn(self.warning)
                self.alert_msg = self.warning
                self.error_flag = True
            else:
                if self.error_flag:
                    self.alert_msg = "error release !!\n\n\n"
                    rospy.loginfo(self.alert_msg)
                    time.sleep(0.5)
                    self.alert_msg = ""
                    rospy.loginfo(self.alert_msg)
                    self.error_flag = False
                else:
                    pass
                pass
            time.sleep(0.1)
        return

if __name__ == "__main__":
    rospy.init_node(node_name)
    al = alert()
    al.thread_start()
    al.check_alert()
