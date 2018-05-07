#!/usr/bin/env python3

import sys
import rospy
import time
import threading
import pyinterface

from necst.msg import String_necst

node_name = 'abs_controller'

class abs_controller(object):
    pro = 0x00
    buff = [1,0,0,0]

    position = ''
    move_position = ''

    board_name = 2724
    rsw_id = 0


    def __init__(self):
        self.sub1 = rospy.Subscriber('hot', String_necst, self.move_pos)
        self.sub2 = rospy.Subscriber('emergency', String_necst, self.emergency)
        self.pub = rospy.Publisher('status_hot', String_necst, queue_size=10, latch = True)
        self.msg = String_necst()
        pass

    def open(self):
        self.dio = pyinterface.open(self.board_name, self.rsw_id)
        self.dio.initialize()
        return

    def start_thread(self):
        get_pos_thread = threading.Thread(target=self.get_pos)
        get_pos_thread.setDaemon(True)
        get_pos_thread.start()
        time.sleep(0.1)
        move_thread = threading.Thread(target=self.move)
        move_thread.setDaemon(True)
        move_thread.start()
        th = threading.Thread(target = self.pub_status)
        th.setDaemon(True)
        th.start()
        return

    def move_pos(self, req):
        self.move_position = req.data.upper()
        return self.move_position

    
    def get_pos(self):
        while not rospy.is_shutdown():
            ret = self.dio.input_byte('IN1_8').to_list()
            print(ret)
            if ret[0] == 0 and ret[1] == 1:#ret == 0x02
                self.position = 'IN'
            elif ret[0] == 1 and ret[1] == 0:#ret == 0x01
                self.position = 'OUT'
            elif ret[0] == 1 and ret[1] == 1:#ret == 0x03
                self.position = 'MOVE'
            else:
                self.print_error('limit error')
                return
            self.msg.data = self.position
            self.msg.from_node = node_name
            time.sleep(0.5)            
        return self.position


    def move(self):
        print('move start')
        while not rospy.is_shutdown():
            print("hot : ", self.position)
            if self.move_position == self.position:
                print('hot is already ' , self.position)
            elif self.move_position != self.position and self.move_position != "":
                if self.move_position == 'IN':
                    self.pro = [0,0,0,0] #0x00
                    self.buff = [1,0,0,0] #0x01
                    print("hot move IN")
                elif self.move_position == 'OUT':
                    self.pro = [0,1,0,0] #0x02
                    self.buff = [1,1,0,0] #0x03
                    print("hot move out")
                else:
                    rospy.logerr("Bad command!!")
                    pass
                self.dio.output_point(self.pro, 1)
                time.sleep(1)
                self.dio.output_point(self.buff,1)
            elif self.move_position == "":
                pass
            else:
                rospy.logerr("Bad command!!")
                pass
            self.move_position = ""
            time.sleep(0.5)
        return

    def pub_status(self):
        while not rospy.is_shutdown():
            self.msg.timestamp = time.time()
            self.pub.publish(self.msg)
            rospy.loginfo(self.msg)
            time.sleep(0.5)
        return


    def emergency(self,req):
        rospy.loginfo('!!!emergency!!!')
        rospy.loginfo('!!!stop azel_move!!!')
        self.emergency_flag = 1
        return
    
if __name__ == '__main__':
    hot = abs_controller()
    hot.open()
    rospy.init_node(node_name)
    rospy.loginfo('waiting publish abs')
    hot.start_thread()
    rospy.spin()

