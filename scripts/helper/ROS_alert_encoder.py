#!/usr/bin/env python3

import sys
import time
import threading

import rospy
from necst.msg import Status_antenna_msg
from necst.msg import Status_encoder_msg
from necst.msg import Alert_msg

node_name = "alert_weather"

class alert(object):

    enc_az = ""
    enc_el = ""
    vel_az = ""
    vel_el = ""    
    
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
        self.sub = rospy.Subscriber("status_antenna", Status_antenna_msg, self.callback_command)
        self.sub = rospy.Subscriber("status_encoder", Status_encoder_msg, self.callback_encoder)        
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
    
    def thread_start(self):
        """ checking alert """
        self.thread_alert = threading.Thread(target=self.pub_status)
        self.thread_alert.setDaemon(True)
        self.thread_alert.start()
        self.thread_sun = threading.Thread(target=self.check_encoder)
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

    def check_encoder(self):
        """
        Warning
        -------
        
        Emergency
        ---------
        If encoder counts is stop, antenna stop.

        """
        while not rospy.is_shutdown():
            warning = ""
            emergency = ""
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
                    emergency += "Emergency : Az encoder can't count!! \n"
                else:
                    az_flag = False
            if el_flag:
                if self.enc_el == start_el:
                    emergency += "Emergency : El encoder can't count!! \n"
                else:
                    el_flag = False

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
