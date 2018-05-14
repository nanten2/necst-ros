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


node_name = "alert_encoder"

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
        self.pub_alert = rospy.Publisher("alert", String_necst, queue_size=10, latch=True)        
        self.pub_antenna = rospy.Publisher("move_stop", Bool_necst, queue_size = 1, latch = True)
        self.pub_dome = rospy.Publisher('dome_move', Dome_msg, queue_size = 10, latch = True)        
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
