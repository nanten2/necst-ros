#!/usr/bin/env python3

"""
create : okuda
correction : kondo 2017/12/19
"""

import sys
import rospy
import time
import threading
import struct

#from necst.msg import Status_m2_msg
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int64


class m2_controller(object):
    #reference  /src/obs/modules/hal/subref.cpp
    #micron is controlled by time
    
    error = []
    m_pos = 0.0
    CW = 0x10 # CW = [0,0,0,0,1,0,0,0]
    CCW = 0x11 #CCW = [1,0,0,0,1,0,0,0]
    LIMIT_CW = 1000
    LIMIT_CCW = -1000
    PULSRATE = 80 #1puls = 0.1micron
    MOTOR_SPEED = 200 # * 10pulses/sec # MOTOR_SPEED_byte = [0,0,0,1,0,0,1,1] # * 10pulses/sec
    m_limit_up = 1
    m_limit_down = 1
    puls = ""
    
    
    def __init__(self):
        self.InitIndexFF()
        self.get_pos()
        
        self.stop_thread = threading.Event()
        self.thread = threading.Thread(target=self.move_thread)
        self.thread.setDaemon(True)
        self.thread.start()

        return

    def start_thread(self):
        th = threading.Thread(target = self.pub_status)
        th.setDaemon(True)
        th.start()
        return
    
    def print_msg(self, msg):
        print(msg)
        return
    
    def print_error(self, msg):
        self.error.append(msg)
        self.print_msg('!!!! ERROR !!!! ' + msg)
        return
    
    def get_pos(self):
        return self.m_pos
    
    def read_pos(self):
        return self.m_pos
    
    def Strobe(self):
        return
    
    def StrobeHOff(self):
        return
    
    
    def move(self, req):
        puls = int(req.data) * self.PULSRATE
        ret = self.get_pos()
        if req.data/1000.+float(ret) <= -4.0 or req.data/1000.+float(ret) >= 5.5:
            self.print_error("move limit")
        if self.m_limit_up == 0 and puls < 0:
            self.print_error("can't move up direction")
        if self.m_limit_down == 0 and puls > 0:
            self.print_error("can't move down direction")
            return
        self.puls = puls
        return

    def move_thread(self):
        while not rospy.is_shutdown():
            if not self.puls:
                pass
            else:
                print("move start")
                self.MoveIndexFF(self.puls)
                self.get_pos()
                self.puls = ""
                print("move end")
            time.sleep(0.1)
            
    
    
    
    def InitIndexFF(self):
        return
    
    def MoveIndexFF(self, puls):
        self.m_pos  += puls/1000
        return 

    def emergency(self,req):
        rospy.loginfo('!!!emergency!!!')
        rospy.loginfo('!!!stop M2!!!')
        self.emergency_flag = 1
        return

    def pub_status(self):
        pub = rospy.Publisher('status_m2',Float64, queue_size=10, latch = True)
        msg = Float64()
        
        while not rospy.is_shutdown():
            pos = self.get_pos()
            msg.data = pos
            pub.publish(msg)
            rospy.loginfo(msg)
            time.sleep(0.5)
        return

if __name__ == '__main__':
    m2 = m2_controller()
    rospy.init_node('m2_controller')
    rospy.loginfo('waiting publish M2')
    m2.start_thread()
    sub = rospy.Subscriber('m2', Int64, m2.move)
    sub = rospy.Subscriber('emergency', String, m2.emergency)
    rospy.spin()
    
"""



def m2_client(host, port):
    client = pyinterface.server_client_wrapper.control_client_wrapper(m2_controller, host, port)
    return client

def m2_monitor_client(host, port):
    client = pyinterface.server_client_wrapper.monitor_client_wrapper(m2_controller, host, port)
    return client

def start_m2_server(port1 = 9999, port2 = 9998):
    m2 = m2_controller()
    server = pyinterface.server_client_wrapper.server_wrapper(m2,'', port1, port2)
    server.start()
    return server

"""
