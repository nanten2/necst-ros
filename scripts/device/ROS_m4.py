#!/usr/bin/env python

import rospy
import time
import threading
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
#import M4
#import board_M4
#import test_board_M4
import pyinterface

from necst.msg import Status_m4_msg
from std_msgs.msg import String

class m4_controller(object):
    speed = 3000
    low_speed = 100
    acc = 500
    dec = 500

    error =[]

    position = ''
    count = 0
    
    shutdown_flag = False

    #M4 = M4.m4_controller()

    def __init__(self):
        pass

    def open(self):
        self.mtr = pyinterface.create_gpg7204(1)
        self.mtr.ctrl.set_limit_config('MTR_LOGIC', 0x000c)
        self.mtr.ctrl.off_inter_lock()
        #self.board_M4 = board_M4.board()
        #self.board_M4 = test_board_M4.board()
        #self.board_M4.set_limit_config('MTR_LOGIC', 0x000c)
        #self.board_M4.off_inter_lock() 
        self.get_pos()
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
    
    '''
    def get_count(self):
        self.count = self.board_M4.get_position()
        return self.count
    '''
    
    def get_pos(self):
        status = self.mtr.ctrl.get_status('MTR_LIMIT_STATUS')
        print(status)
        if status == 0x0004:
            #SMART
           self.position = 'OUT'
        elif status == 0x0008:
            #NAGOYA
            self.position = 'IN'
        elif status == 0x0000:
            self.position = 'MOVE'
        else:
            self.print_error('limit error')
            return
        return self.position

    def move(self,req):
        print('move start')
        pos = self.get_pos()

        if req.data == pos:
            if req.data.upper() == 'OUT':
                self.print_msg('m4 is alrady out')
                return
            elif req.data.upper() == 'IN':
                self.print_msg('m4 is alrady in')
                return
            else:
                self.print_msg('m4 is alrady move')
                return
        else:
            if req.data.upper() == 'OUT':
                nstep = 60500
                self.print_msg('m4 move out')
            elif req.data.upper() == 'IN':
                nstep = -60500
                self.print_msg('m4 move in')
            else:
                self.print_error('parameter error')
                return
            self.mtr.move(self.speed, nstep, self.low_speed, self.acc, self.dec)
            #time.sleep(12.)
            #count = self.get_count()
        pos= self.get_pos()
        return

    def m4_out(self):
        self.move('OUT')
        return
    
    def m4_in(self):
        self.move('IN')
        return
    
    def stop(self):
        self.mtr.stop()
        return
    
    def read_pos(self):
        return self.position
    
    def read_count(self):
        return self.count

    def test(self):
        return


    def emergency(self,req):
        rospy.loginfo('!!!emergency!!!')
        rospy.loginfo('!!!stop M4!!!')
        self.emergency_flag = 1
        return

    def pub_status(self):
        pub = rospy.Publisher('status_m4',Status_m4_msg, queue_size=10, latch = True)
        msg = Status_m4_msg()

        while not rospy.is_shutdown():
            #pos = self.get_pos()
            msg.m4_position = self.position
            pub.publish(msg)
            rospy.loginfo(self.position)
            time.sleep(0.5)
        return

if __name__ == '__main__':
    m4 = m4_controller()
    m4.open()
    rospy.init_node('m4_controller')
    rospy.loginfo('waiting publish M4')
    m4.start_thread()
    sub = rospy.Subscriber('m4', String, m4.move)
    sub = rospy.Subscriber('emergency', String, m4.emergency)
    rospy.spin()
"""
def m4_client(host,port):
    client = pyinterface.server_client_wrapper.control_client_wrapper(m4_controller, host, port)
    return client

def m4_monitor_client(host,port):
    client = pyinterface.server_client_wrapper.monitor_client_wrapper(m4_controller, host, port)
    return client

def start_m4_server(port1=6003, port2=6004):
    m4 = m4_controller()
    server = pyinterface.server_client_wrapper.server_wrapper(m4,'', port1, port2)
    server.start()
    return server
"""
