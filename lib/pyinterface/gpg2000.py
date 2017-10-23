#!/usr/bin/env python
import rospy
from necst.msg import Status_encoder_msg

class gpg2000(object):
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
    

    def __init__(self, ndev=1, remote=False):
        initialize = not remote
        self.ctrl = gpg2000_controller(ndev, initialize=initialize)
        pass

    def di_check(self, start_num, num):
        ret = self.ctrl.in_point(start_num, num)
        return ret

    def do_output(self, buffer, startnum, num):
        self.ctrl.out_point(buffer, startnum, num)
        return




        
        
class gpg2000_controller(object):

    def __init__(self, ndev=1, boardid=2724, initialize=True):
        self.pub = rospy.Publisher("pyinterface", Status_encoder_msg, queue_size=10, latch=True)
        self.ndev = ndev
        self.az = 0
        self.el = 0
        
    def out_word(self, name, value):
        if self.ndev == 3:
            if name == "FBIDIO_OUT1_16":
                self.az = value
            elif name == "FBIDIO_OUT17_32":
                print("el")
                self.el = value
            else:
                pass
            vel = Status_encoder_msg()
            vel.enc_az = self.az
            vel.enc_el = self.el
            self.pub.publish(vel)
        else:
            pass

    def in_point(self, startnum, num):
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

    def out_point(self, buffer, startnum, num):
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
            global dome_enc_1
            if buffer[0] == 0:
                self.turn = 'right'
            else:
                self.turn = 'left'
                
            if buffer[2:4] == [0,0]:
                self.speed = 'low'#for dummy enc publish
            elif buffer[2:4] == [1,0]:
                self.speed = 'mid'#for dummy enc publish
            elif buffer[2:4] == [0,1]:
                self.speed = 'high'#for dummy enc publish
            else:
                self.speed = 'None'#for dummy enc publish

            c_dome_enc = dome_enc_1
            calc_dome_enc = c_dome_enc
            if self.turn == 'right':
                if self.speed == 'high':
                    calc_dome_enc = c_dome_enc + 10000
                elif self.speed == 'mid':
                    calc_dome_enc = c_dome_enc + 4000
                elif self.speed == 'low':
                    calc_dome_enc = c_dome_enc + 1000
                print('%%%')
            if self.turn == 'left':
                if self.speed == 'high':
                    calc_dome_enc = c_dome_enc - 10000
                elif self.speed == 'mid':
                    print('###')
                    calc_dome_enc = c_dome_enc - 4000
                elif self.speed == 'low':
                    calc_dome_enc = c_dome_enc - 1000

            
            dome_enc_1 = calc_dome_enc
            print(self.turn, self.speed, calc_dome_enc, dome_enc_1)
            time.sleep(0.5)
            pass
        return
