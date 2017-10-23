#!/usr/bin/env python

import rospy
import time
import threading
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
#import abs
#import board_abs
import test_board_abs

from std_msgs.msg import String 
from necst.msg import Status_hot_msg

class abs_controller(object):
    #abs = abs.abs_controller()
    pro = 0x00
    buff = 0x00
    error = []

    position = ''


    def __init__(self):
        pass

    def open(self):
        #self.board_abs = board_abs.board()
        self.board_abs = test_board_abs.board()
        self.get_pos()
        return

    def start_thread(self):
        th = threading.Thread(target = self.pub_status)
        th.setDaemon(True)
        th.start()
        return
    

    def test(self):
        return

    def print_msg(self, msg):
        print(msg)
        return
        
    def print_error(self, msg):
        self.error.append(msg)
        self.print_msg('!!!! ERROR !!!! ' + msg)
        return

    '''
    def pos_tel(self):
        ret = self.board_abs.in_byte('FBIDIO_IN1_8')

if ret == 0x02:
            print('position : IN')
            elif ret == 0x01:   
                print('position : OUT')
                elif ret == 0x03:   
                    print('position : MOVE')
                    else:
                        self.print_error('limit error')
            print(ret)
        return
    '''

    def get_pos(self):
        ret = self.board_abs.in_byte('FBIDIO_IN1_8')
        
        if ret == 0x02:
            self.position = 'IN'
        elif ret == 0x01:
            self.position = 'OUT'
        elif ret == 0x03:
            self.position = 'MOVE'
        else:
            self.print_error('limit error')
            return
        return self.position



    def move(self,req):
        print('move start')
        pos = self.get_pos()
        print(pos)
        if pos == req.data:
            print('hot is already ' + req.data)
            return
        if req.data == 'IN':
            #self.pro = 0x00
            self.buff = 0x01
        elif req.data == 'OUT':
            #self.pro = 0x02
            self.buff = 0x03
        print(req.data)
        #self.board_abs.out_byte('FBIDIO_OUT1_8', self.pro)
        #time.sleep(1)
        self.board_abs.out_byte('FBIDIO_OUT1_8', self.buff)
        time.sleep(5)
        self.get_pos()
        return

    def emergency(self,req):
        rospy.loginfo('!!!emergency!!!')
        rospy.loginfo('!!!stop azel_move!!!')
        self.emergency_flag = 1
        return

    def pub_status(self):
        pub = rospy.Publisher('status_hot', Status_hot_msg, queue_size=10, latch = True)
        msg = Status_hot_msg()

        while not rospy.is_shutdown():
            pos = self.get_pos()
            msg.hot_position =pos
            pub.publish(msg)
            rospy.loginfo(pos)
            time.sleep(0.5)
        return

    def move_r(self):
        self.move('IN')
        return

    def move_sky(self):
        self.move('OUT')
        return

    '''
    def stop(self):
    self.buff = 0x04
    self.board_abs.out_byte('FBIDIO_OUT1_8', self.buff)
return

    '''


if __name__ == '__main__':
    abs = abs_controller()
    abs.open()
    rospy.init_node('abs_controller')
    rospy.loginfo('waiting publish abs')
    abs.start_thread()
    sub = rospy.Subscriber('hot', String, abs.move)
    sub = rospy.Subscriber('emergency', String, abs.emergency)
    rospy.spin()

def abs_client(host, port):
    client = pyinterface.server_client_wrapper.control_client_wrapper(abs_controller, host, port)
    return client

def abs_monitor_client(host, port):
    client = pyinterface.server_client_wrapper.monitor_client_wrapper(abs_controller, host, port)
    return client

def start_abs_server(port1 = 6001, port2 = 6002):
    abs = abs_controller()
    server = pyinterface.server_client_wrapper.server_wrapper(abs,'', port1, port2)
    server.start()
    return server
