#!/usr/bin/env python
"""
this script is dummy for ROS_dome.py
2017/10/01 shiotani

"""
#import pyinterface#N
import time
import rospy ###for dummy dome
#import ros_start
import threading
from necst.msg import Dome_msg
from std_msgs.msg import String
import pyinterface
#import shiotani_pyinterface

class dome_board(object):
    ###dome
    right_act = 'OFF'
    right_pos = 'CLOSE' 
    left_act = 'OFF'
    left_pos = 'CLOSE'
    ###memb
    memb_act = 'OFF'
    memb_pos = 'CLOSE'

    ###get_action/move_status
    move_status = 'OFF'

    ###remote/local
    status = 'LOCAL'

    speed = 'None'
    turn = 'None'
    
    

    def __init__(self):
        self.dio = pyinterface.create_gpg2000(5)#N
        #self.dio = shiotani_pyinterface.create_gpg2000(5)#N
        pass

    def start_thread(self):
        th = threading.Thread(target = self.pub_enc)
        th.setDaemon(True)
        th.start()
        pass

    def pub_enc(self):
        pub = rospy.Publisher('dome_enc1', String, queue_size = 10, latch = True)
        pub2 = rospy.Publisher('dome_enc2', String, queue_size = 10, latch = True)
        enc = String()
        enc.data = self.speed
        turn = String()
        turn.data = self.turn
        pub.publish(enc)
        pub2.publish(turn)
        rospy.loginfo('publish speed turn')
        print(self.speed, self.turn)
        rospy.logwarn('check_dome_board_param')
        print(self.right_act, self.right_pos, self.left_act, self.left_pos, self.memb_act, self.memb_pos, self.move_status)

    def do_output(self, buffer, startnum, num):
        pub = rospy.Publisher('dome_dummy', Dome_msg, queue_size = 10, latch = True)
        print('do_output', buffer, startnum, num)
        return self.dio.do_output(buffer, startnum, num)#N

        ###for dummy
        """

        if startnum == 2 and num == 1:#end thread(ROS_dome.py)
            self.move_status = 'OFF'
            print('dome_stop function called dome_board.py')
            self.speed = 'None'
            self.turn = 'None'
            
            return
            
        if startnum == 5 and num == 2:
            if buffer == [0, 0]:
                #self.right_pos = 'MOVE'
                #self.left_pos = 'MOVE'
                #time.sleep(2)
                #self.right_pos = 'OPEN'
                #self.left_pos = 'OPEN'
                pass
                
            elif buffer == [1, 1]:
                self.right_pos = 'MOVE'
                self.left_pos = 'MOVE'
                time.sleep(2)
                self.right_pos = 'OPEN'
                self.left_pos = 'OPEN'
                pass
            elif buffer == [0, 1]:
                self.right_pos = 'MOVE'
                self.left_pos = 'MOVE'
                time.sleep(2)
                self.right_pos = 'CLOSE'
                self.left_pos = 'CLOSE'
                

        if startnum == 7 and num == 2:
            if buffer == [1, 1]:
                self.memb_pos = 'OPEN'
            elif buffer == [0,0]:
                pass
            elif buffer == [0, 1]:
                self.memb_pos = 'CLOSE'

        if startnum == 9 and num == 2:
            if buffer == [1, 1]:
                pass
            else:#buffer == [0, 0]
                pass
            
        if startnum == 1 and num == 6:### need to add 'turn'
            if buffer[0] == 0:
                self.turn = 'right'
            else:
                self.turn = 'left'
                
            if buffer[2:4] == [0,0]:
                speed = 'low'
                self.speed = 'low'#for dummy enc publish
            elif buffer[2:4] == [1,0]:
                speed = 'mid'
                self.speed = 'mid'#for dummy enc publish
            elif buffer[2:4] == [0,1]:
                speed = 'high'
                self.speed = 'high'#for dummy enc publish
            else:
                speed = 'None'###this code is not needed
                self.speed = 'None'#for dummy enc publish
                
            rospy.loginfo(speed)
            time.sleep(0.5)
            pass

        """


    def di_check(self, start_num, num):
        return self.dio.di_check(start_num, num)#N
        #print('di_check')
        """dummy script
        for  simulator"""
        """
        if start_num == 1 and num ==1:
            if self.move_status == 'OFF':
                return 0
            else:
                return 1#DRIVE
        
        if start_num == 2 and num == 6:
            ret = [0]*6
            if self.right_act == 'OFF':
                ret[0] = 0
            else:
                ret[0] = 1

            if self.right_pos == 'OPEN':
                ret[1] = 1
            elif self.right_pos == 'MOVE':
                ret[1] = 0
                ret[2] = 0
            elif self.right_pos == 'CLOSE':
                ret[1] = 0
                ret[2] = 1
                
            if self.left_act == 'OFF':
                ret[3] = 0
            else:
                ret[3] = 1
                
            if self.left_pos == 'OPEN':
                ret[4] = 1
            elif self.left_pos == 'MOVE':
                ret[4] = 0
                ret[5] = 0
            elif self.left_pos == 'CLOSE':
                ret[4] = 0
                ret[5] = 1
            return ret

        if start_num == 8 and num == 3: #get_memb_status
            ret = [0]*3
            if self.memb_act == 'OFF':
                ret[0] = 0
            else:
                ret[0] = 1 #DRIVE

            if self.memb_pos == 'OPEN':
                ret[1] = 1 #OPEN
            elif self.memb_pos == 'MOVE':
                ret[1] = 0
                ret[2] = 0
            elif self.memb_pos == 'CLOSE':
                ret[1] = 0
                ret[2] = 1

            return ret

        if start_num == 11 and num == 1: #get_remote_status
            if self.status == 'REMOTE':
                return [0]
            elif self.status == 'LOCAL':
                return[1]

        if start_num == 16 and num == 6: #error_check
            ret = [0]*6
            return ret

        if start_num ==12 and num ==4: #_limit_check
            return [0,0,0,0]
        """

    def pub_status(self):
        pub1 = rospy.Publisher('dome_enc1', String, queue_size = 10, latch = True)
        pub2 = rospy.Publisher('dome_enc2', String, queue_size = 10, latch = True)
        d = String()
        d1 = String()
        d.data = self.speed
        d1.data = self.turn
        pub.publish(d)
        pub.publish(d1)
        
        
        
            
            

if __name__ == '__main__':
    rospy.init_node('dome_board')
    d = dome_board()
    d.start_thread()


                
