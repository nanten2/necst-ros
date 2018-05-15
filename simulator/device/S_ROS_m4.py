#!/usr/bin/env python3

"""
create : okuda
last update : 2017/12/17 kondo
"""

import sys
import rospy
import time
import threading

from necst.msg import String_necst

node_name = 'm4_controller'

class m4_controller(object):
    speed = 3000
    low_speed = 100
    acc = 500
    dec = 500

    error =[]

    position = 'IN'
    move_position = ''
    count = 0
    
    shutdown_flag = False


    def __init__(self):
        pass

    def open(self):
        self.get_pos()
        return

    def counter_reset(self):
        position = self.position
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
    
    def get_pos(self):
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
                    self.position = 'MOVE'
                    time.sleep(2)
                    self.position = 'SMART'
                    self.print_msg('m4 move out')
                elif self.move_position == 'NAGOYA':
                    #nstep = -60500
                    step=-1
                    self.position = 'MOVE'
                    time.sleep(2)
                    self.position = 'NAGOYA'
                    self.print_msg('m4 move in')
                else:
                    self.print_error('parameter error')
                    pass
                #self.mtr.set_motion(mode="JOG",step=step)
                #self.mtr.start_motion(mode="JOG")
            elif self.move_position == "":
                pass
            else:
                rospy.logerr("Bad command!!")
                pass
            self.move_position = ""
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
    rospy.init_node(node_name)    
    m4 = m4_controller()
    m4.open()
    rospy.loginfo('waiting publish M4')
    m4.start_thread()
    sub = rospy.Subscriber('m4', String_necst, m4.move_pos)
    sub = rospy.Subscriber('emergency', String_necst, m4.emergency)
    rospy.spin()

