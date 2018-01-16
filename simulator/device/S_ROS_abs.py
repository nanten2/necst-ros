#!/usr/bin/env python3

import sys
import rospy
import time
import threading
from std_msgs.msg import String 
from necst.msg import Status_hot_msg

class abs_controller(object):
    pro = 0x00
    buff = [1,0,0,0]
    error = []

    position = 'IN'
    move_position = ''

    board_name = 2724
    rsw_id = 0


    def __init__(self):
        pass

    def open(self):
        self.get_pos()
        return

    def start_thread(self):
        th = threading.Thread(target = self.pub_status)
        th.setDaemon(True)
        th.start()
        move_thread = threading.Thread(target=self.move)
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
        #ret = self.dio.input_byte('IN1_8').to_list()
        #print(ret)
        """
        if ret[0] == 0 and ret[1] == 1:#ret == 0x02
            self.position = 'IN'
        elif ret[0] == 1 and ret[1] == 0:#ret == 0x01
            self.position = 'OUT'
        elif ret[0] == 1 and ret[1] == 1:#ret == 0x03
            self.position = 'MOVE'
        else:
            self.print_error('limit error')
            return
        """
        return self.position

    def move_pos(self, req):
        self.move_position = req.data.upper()
        return self.move_position

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
                    self.position = 'MOVE'
                    time.sleep(1)
                    self.position = 'IN'
                    print("hot move IN")
                elif self.move_position == 'OUT':
                    self.pro = [0,1,0,0] #0x02
                    self.buff = [1,1,0,0] #0x03
                    self.position = 'MOVE'
                    time.sleep(1)
                    self.position = 'OUT'
                    print("hot move out")
                else:
                    rospy.logerr("Bad command!!")
                    pass
                time.sleep(1)
            elif self.move_position == "":
                pass
            else:
                rospy.logerr("Bad command!!")
                pass
            self.move_position = ""
            time.sleep(0.5)
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
            print(pos)
            msg.hot_position = pos
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


if __name__ == '__main__':
    hot = abs_controller()
    hot.open()
    rospy.init_node('abs_controller')
    rospy.loginfo('waiting publish abs')
    hot.start_thread()
    sub = rospy.Subscriber('hot', String, hot.move_pos)
    sub = rospy.Subscriber('emergency', String, hot.emergency)
    rospy.spin()
