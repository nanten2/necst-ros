#!/usr/bin/env python3

"""
create : okuda
last update : 2017/12/17 kondo
"""

import sys
import rospy
import time
import threading
import pyinterface

from necst.msg import String_necst

node_name = 'm4_controller'

class m4_controller(object):
    speed = 3000
    low_speed = 100
    acc = 500
    dec = 500

    error =[]

    position = ''
    move_position = ''
    count = 0
    
    shutdown_flag = False

    #M4 = M4.m4_controller()

    def __init__(self):
        pass

    def open(self):
        #self.mtr = pyinterface.create_gpg7204(1)
        self.mtr = pyinterface.open(7204,0) #test rsw
        self.mtr.initialize()
        #self.mtr.ctrl.set_limit_config('MTR_LOGIC', 0x000c)
        self.mtr.set_limit_config('LOGIC', '+EL -EL', axis=1)
        #self.mtr.ctrl.off_inter_lock() # abolition
        #self.board_M4 = board_M4.board()
        #self.board_M4 = test_board_M4.board()
        #self.board_M4.set_limit_config('MTR_LOGIC', 0x000c)
        #self.board_M4.off_inter_lock() 
        self.get_pos()
        return

    def counter_reset(self):
        is_stop = not self.mtr.get_status(axis=1)['busy']
        position = self.position
        
        if (position=='NAGOYA') & is_stop:
            self.mtr.set_counter(0, axis=1)
            pass
        return

    def start_thread(self):
        th = threading.Thread(target = self.pub_status)
        th.setDaemon(True)
        th.start()
        move_thread = threading.Thread(target = self.move)
        move_thread.setDaemon(True)
        move_thread.start()
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
        status = self.mtr.get_status()
        #print(status)
        if status["busy"] == True:
            self.position = 'MOVE'
        elif status["limit"]["+EL"] == 0: #status == 0x0004:
            #OUT
            self.position = 'SMART'
        elif status["limit"]["-EL"] == 0:
            #IN
            self.position = 'NAGOYA'
        else:
            self.print_error('limit error')
            return
        return self.position

    def move_pos(self, req):
        if req.data.upper() == "IN":
            self.move_position = "NAGOYA"
        elif req.data.upper() == "OUT":
            self.move_position = "SMART"
        else:
            rospy.logerr("!!command error!!")
            pass
        return

    def move(self):
        print('move start')
        while not rospy.is_shutdown():
            print("M4 : ", self.position)
            if self.move_position == self.position:
                print('M4 is already ' , self.position)
            elif self.move_position != self.position and self.move_position != "":
                if self.move_position == 'SMART':
                    #nstep = 60500
                    step=1
                    self.print_msg('m4 move out')
                elif self.move_position == 'NAGOYA':
                    #nstep = -60500
                    step=-1
                    self.print_msg('m4 move in')
                else:
                    self.print_error('parameter error')
                    pass
                #self.mtr.move(self.speed, nstep, self.low_speed, self.acc, self.dec)
                self.mtr.set_motion(mode="JOG",step=step)
                self.mtr.start_motion(mode="JOG")
            elif self.move_position == "":
                pass
            else:
                rospy.logerr("Bad command!!")
                pass
            self.move_position = ""
            if self.mtr.get_status()['busy'] == False:
                self.counter_reset()
            else:
                pass
            time.sleep(0.5)
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
        pub = rospy.Publisher('status_m4',String_necst, queue_size=10, latch = True)
        msg = String_necst()
        while not rospy.is_shutdown():
            pos = self.get_pos()
            msg.data = self.position
            msg.from_node = node_name
            msg.timestamp = time.time()
            pub.publish(msg)
            rospy.loginfo(self.position)
            time.sleep(0.5)
        return

if __name__ == '__main__':
    m4 = m4_controller()
    m4.open()
    rospy.init_node(node_name)
    rospy.loginfo('waiting publish M4')
    m4.start_thread()
    sub = rospy.Subscriber('m4', String_necst, m4.move_pos)
    sub = rospy.Subscriber('emergency', String_necst, m4.emergency)
    rospy.spin()
