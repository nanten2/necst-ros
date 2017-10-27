#!/usr/bin/env python
import time
import rospy
from necst.msg import Status_encoder_msg

class gpg2000(object):    

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
        self.drive = 0
        self.contactor = 0
        self.az = 0
        self.el = 0

        ###dome
        self.right_act = 'OFF'
        self.right_pos = 'CLOSE' 
        self.left_act = 'OFF'
        self.left_pos = 'CLOSE'
        ###memb
        self.memb_act = 'OFF'
        self.memb_pos = 'CLOSE'
        
        ###get_action/move_status
        self.move_status = 'OFF'
        
        ###remote/local
        self.status = 'LOCAL'
        
        self.speed = 'None'
        self.turn = 'None'

    def out_word(self, name, value):
        self.read_enc()
        if name == "FBIDIO_OUT1_16":
            self.enc_az = value
        elif name == "FBIDIO_OUT17_32":
            print("el")
            self.enc_el = value
        else:
            pass
        vel = Status_encoder_msg()
        vel.enc_az = self.enc_az
        vel.enc_el = self.enc_el
        self.pub.publish(vel)
        self.write_enc()

    def write_enc(self):
        with open("/home/amigos/ros/src/necst/lib/"+"enc.txt","w") as ef:
            ef.write(str(self.enc_az))
            ef.write("\n")
            ef.write(str(self.enc_el))
        return

    def read_enc(self):
        with open("/home/amigos/ros/src/necst/lib/"+"enc.txt","r") as ref:
            txt = ref.readlines()
            txt = [txt[i].split()[0] for i in range(len(txt))]
            self.enc_az = float(txt[0])
            self.enc_el = float(txt[1])
        return txt
            
   
        
    def out_point(self, buffer, startnum, num):
        self.dome_read()
        if startnum == 2 and num == 1:#end thread(ROS_dome.py)
            self.move_status = 'OFF'
            print('dome_stop function called dome_board.py')
            self.speed = 'None'
            self.turn = 'None'
            self.dome_write()
            return
            
        if startnum == 5 and num == 2:
            if buffer == [0, 0]:
                return
                
            elif buffer == [1, 1]:
                self.right_pos = 'MOVE'
                self.left_pos = 'MOVE'
                self.dome_write()
                time.sleep(2)
                self.right_pos = 'OPEN'
                self.left_pos = 'OPEN'
                self.dome_write()
                return
            elif buffer == [0, 1]:
                self.right_pos = 'MOVE'
                self.left_pos = 'MOVE'
                self.dome_write()
                time.sleep(2)
                self.right_pos = 'CLOSE'
                self.left_pos = 'CLOSE'
                self.dome_write()
                return

            return
                

        if startnum == 7 and num == 2:
            if buffer == [1, 1]:
                self.memb_pos = 'OPEN'
                self.dome_write()
                return
            elif buffer == [0,0]:
                return
            elif buffer == [0, 1]:
                self.memb_pos = 'CLOSE'
                self.dome_write()
                return

            return

        if startnum == 9 and num == 2:
            if buffer == [1, 1]:
                self.dome_write()
                return
            else:#buffer == [0, 0]
                self.dome_write()
                eturn

            return
            
        if startnum == 1 and num == 6:### need to add 'turn'
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

            c_dome_enc = self.dome_enc_read()
            c_dome_enc = float(c_dome_enc)

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

            with open("/home/amigos/ros/src/necst/lib/"+"dome_enc.txt","w") as wf:
                wf.write(str(calc_dome_enc))
            #print(self.turn, self.speed, calc_dome_enc, dome_enc_1)
            time.sleep(0.5)
            self.dome_write()
            return

    def in_point(self, start_num, num):
        status = self.dome_read()
        if start_num == 1 and num ==1:
            if status[0] == 'OFF':
                return 0
            else:
                return 1#DRIVE
        
        if start_num == 2 and num == 6:
            ret = [0]*6
            if status[1] == 'OFF':
                ret[0] = 0
            else:
                ret[0] = 1

            if status[3] == 'OPEN':
                ret[1] = 1
            elif status[3] == 'MOVE':
                ret[1] = 0
                ret[2] = 0
            elif status[3] == 'CLOSE':
                ret[1] = 0
                ret[2] = 1
                
            if status[2] == 'OFF':
                ret[3] = 0
            else:
                ret[3] = 1
                
            if status[4] == 'OPEN':
                ret[4] = 1
            elif status[4] == 'MOVE':
                ret[4] = 0
                ret[5] = 0
            elif status[4] == 'CLOSE':
                ret[4] = 0
                ret[5] = 1
            return ret

        if start_num == 8 and num == 3: #get_memb_status
            ret = [0]*3
            if status[5] == 'OFF':
                ret[0] = 0
            else:
                ret[0] = 1 #DRIVE

            if status[6] == 'OPEN':
                ret[1] = 1 #OPEN
            elif status[6] == 'MOVE':
                ret[1] = 0
                ret[2] = 0
            elif status[6] == 'CLOSE':
                ret[1] = 0
                ret[2] = 1

            return ret

        if start_num == 11 and num == 1: #get_remote_status
            if status[7] == 'REMOTE':
                return [0]
            elif status[7] == 'LOCAL':
                return[1]

        if start_num == 16 and num == 6: #error_check
            ret = [0]*6
            return ret

        if start_num ==12 and num ==4: #_limit_check
            return [0,0,0,0]


    def dome_write(self):
        with open("/home/amigos/ros/src/necst/lib/"+"dome.txt","w") as f:
            f.write(self.move_status)
            f.write("\n")
            f.write(self.right_act)
            f.write("\n")
            f.write(self.left_act)
            f.write("\n")
            f.write(self.right_pos)
            f.write("\n")
            f.write(self.left_pos)
            f.write("\n")
            f.write(self.memb_act)
            f.write("\n")
            f.write(self.memb_pos)
            f.write("\n")
            f.write(self.status)
        return

    def dome_read(self):
        with open("/home/amigos/ros/src/necst/lib/"+"dome.txt","r") as ref:
            txt = ref.readlines()
            txt = [txt[i].split()[0] for i in range(len(txt))]
            self.move_status = txt[0]
            self.right_act = txt[1]
            self.left_act = txt[2]
            self.right_pos = txt[3]
            self.left_pos = txt[4]
            self.memb_act = txt[5]
            self.memb_pos = txt[6]
            self.status = txt[7]
        return txt
      

    def dome_enc_read(self):
        with open("/home/amigos/ros/src/necst/lib/"+"dome_enc.txt","r") as rf:
            txt = rf.readlines()
            txt = txt[0].split()[0]
            print("#################")
            print(txt)
            print("###################")

        return float(txt)

    def out_byte(self, name, value):
        if self.ndev == 10:
            if name == "FBIDIO_OUT1_8":
                if value == 1:
                    pos = "in"
                elif value == 3:
                    pos = "out"
                else:
                    pos = "move"
            else:
                pass
            with open("/home/amigos/ros/src/necst/lib/"+"hot.txt","w") as f:
                f.write(str(pos))
        else:
            dr = self.drive_read()
            if name == "FBIDIO_OUT1_8":
                if value == 3:
                    self.drive = 1
                elif value == 0:
                    self.drive = 0
            elif name == "FBIDIO_OUT9_16":
                if value == 15:
                    self.contactor = 1
                elif value == 0:
                    self.contactor = 0
            else:
                pass
            self.drive_write()
        return
            
        
            
    def in_byte(self, no):
        if self.ndev == 10:
            with open("/home/amigos/ros/src/necst/lib/"+"hot.txt","r") as f:
                txt = f.readlines()
            hot = txt[0].split()[0]
            if hot == "in":
                value = 2
            elif hot == "out":
                value =1
            elif hot == "move":
                value = 3
        else:
            with open("/home/amigos/ros/src/necst/lib/"+"drive.txt","r") as f:
                txt = f.readlines()
            dr = int(txt[0].split()[0])
            co = int(txt[1].split()[0])
            if no == 'FBIDIO_IN1_8':
                if dr == 1 and co == 1:
                    value = int(b"11111111",2)
                elif dr == 0 and co == 1:
                    value = int(b"11111100",2)
                elif dr == 1 and co == 0:
                    value = int(b"11110011",2)
                elif dr == 0 and co == 0:
                    value = int(b"11110000",2)                
            elif no == 'FBIDIO_IN9_16':
                value = int(b"11111111",2)
            elif no == 'FBIDIO_IN17_24':
                value = int(b"11111111",2)
            elif no == 'FBIDIO_IN25_32':
                value = int(b"11111111",2)
            else:
                print("############ !!error!! ############")

        
        return value

    def drive_write(self):
        with open("/home/amigos/ros/src/necst/lib/"+"drive.txt","w") as f:
            f.write(str(self.drive))
            f.write("\n")
            f.write(str(self.contactor))
        return

    def drive_read(self):
        with open("/home/amigos/ros/src/necst/lib/"+"drive.txt","r") as df:
            txt = df.readlines()
            txt = [txt[i].split()[0] for i in range(len(txt))]
            print(txt)
            self.drive = int(txt[0])
            self.contactor = int(txt[1])
            print(self.drive)
            print(self.contactor)
        return txt
