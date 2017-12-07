#!/usr/bin/env python3

import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
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
        self.dir = "/home/necst/ros/src/necst/lib/"
      
        self.speed = 'None'
        self.turn = 'None'

        #self.dome = [move_status, right_act, left_act, right_pos, left_pos, memb_act, memb_pos, status]
        self.dome = ['OFF', 'OFF', 'OFF', 'CLOSE', 'CLOSE', 'OFF', 'CLOSE', 'LOCAL']

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
        with open(self.dir+"enc.txt","w") as ef:
            ef.write(str(self.enc_az))
            ef.write("\n")
            ef.write(str(self.enc_el))
        return

    def read_enc(self):
        with open(self.dir+"enc.txt","r") as ref:
            txt = ref.readlines()
            txt = [txt[i].split()[0] for i in range(len(txt))]
            self.enc_az = float(txt[0])
            self.enc_el = float(txt[1])
        return txt
            
   
        
    def out_point(self, buffer, startnum, num):
        self.dome_read()
        dome_list = self.dome
        if startnum == 2 and num == 1:#end thread(ROS_dome.py)
            dome_list[7] = 'OFF'
            print('dome_stop function called dome_board.py')
            self.speed = 'None'
            self.turn = 'None'
            self.dome_write()
            return
            
        if startnum == 5 and num == 2:
            if buffer == [0, 0]:
                return
                
            elif buffer == [1, 1]:
                dome_list[3] = 'MOVE'
                dome_list[4] = 'MOVE'
                self.dome = dome_list
                self.dome_write()
                time.sleep(4)
                dome_list[3] = 'OPEN'
                dome_list[4] = 'OPEN'
                self.dome = dome_list
                self.dome_write()
                return
            elif buffer == [0, 1]:
                dome_list[3] = 'MOVE'
                dome_list[4] = 'MOVE'
                self.dome = dome_list
                self.dome_write()
                time.sleep(4)
                dome_list[3] = 'CLOSE'
                dome_list[4] = 'CLOSE'
                self.dome = dome_list
                self.dome_write()
                return

            return
                

        if startnum == 7 and num == 2:
            if buffer == [0,0]:
                return
            elif buffer == [1, 1]:
                dome_list[6] = 'MOVE'
                self.dome = dome_list
                self.dome_write()
                time.sleep(4)
                dome_list[6] = 'OPEN'
                self.dome = dome_list
                self.dome_write()
                return
            elif buffer == [0, 1]:
                dome_list[6] = 'MOVE'
                self.dome = dome_list
                self.dome_write()
                time.sleep(4)
                dome_list[6] = 'CLOSE'
                self.dome = dome_list
                self.dome_write()
                return

            return

        if startnum == 9 and num == 2:
            if buffer == [1, 1]:
                return
            else:#buffer == [0, 0]
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
            if self.turn == 'left':
                if self.speed == 'high':
                    calc_dome_enc = c_dome_enc - 10000
                elif self.speed == 'mid':
                    calc_dome_enc = c_dome_enc - 4000
                elif self.speed == 'low':
                    calc_dome_enc = c_dome_enc - 1000

            with open("dome_enc.txt","w") as wf:
                wf.write(str(calc_dome_enc))
            time.sleep(0.5)
            return

    def in_point(self, start_num, num):
        status = self.dome_read()
        print("############################3")
        print(status)
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
        _list = ''
        for i in range(len(self.dome)):
            _list += self.dome[i]+"\n"
        with open(self.dir+"dome.txt","w") as f:
            f.write(_list)
        return

    def dome_read(self):
        while True:
            with open(self.dir+"dome.txt","r") as ref:
                txt = ref.readlines()
                self.dome = [txt[i].split()[0] for i in range(len(txt))]
            if len(self.dome) == 10:
                break
            print("##########################")
            print(self.dome)
        return self.dome
      

    def dome_enc_read(self):
        with open(self.dir+"dome_enc.txt","r") as rf:
            txt = rf.readlines()
            txt = txt[0].split()[0]
        return float(txt)

    def out_byte(self, name, value):
        if self.ndev == 10:#hot
            if name == "FBIDIO_OUT1_8":
                if value == 1:
                    pos = "in"
                elif value == 3:
                    pos = "out"
                else:
                    pos = "move"
            else:
                pass
            with open(self.dir+"hot.txt","w") as f:
                f.write(str(pos))
        elif self.ndev == 2:#m2
            with open(self.dir+"m2.txt","r") as f:
                txt = f.readlines()
                txt = [txt[i].split()[0] for i in range(len(txt))]
                m2_move = txt[0]
                m2_dir = int(txt[1])
                m2_pos = float(txt[2])
                m2_param = float(txt[3])
            if name == "FBIDIO_OUT1_8":
                if value == 0x08:
                    m2_move = True
                if m2_move and m2_param == 1000:
                    if value == 0x08:
                        pass
                    elif value == 0xff:
                        pass
                    elif value == 0xc0:
                        pass
                    elif value == 0x10:
                        m2_dir = +1
                    elif value == 0x11:
                        m2_dir = -1
                    elif value == 0x18:
                        m2_move = False
                    elif value == 0x48:
                        pass
                    elif value == 0x05:
                        pass
                    elif value == 0x40:
                        pass
                    elif value == 200:
                        pass
                    elif value == 0x50:
                        pass
                    elif value == 100:
                        pass
                    elif value == 0:
                        pass
                    else:
                        m2_param = value
                elif m2_move and m2_param != 1000:
                        m2_pos += (m2_param*256 + value)/80.*m2_dir
                        m2_param = 1000
                else:
                    print("############## error #################")
            elif name == "FBIDIO_OUT9_16":
                pass
            else:
                print("############## error #################")
            _list = [str(m2_move)+"\n"+str(m2_dir)+"\n"+str(m2_pos)+"\n"+str(m2_param)]
            with open(self.dir+"m2.txt","w") as f:
                f.write(_list)

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
            with open(self.dir+"hot.txt","r") as f:
                txt = f.readlines()
            hot = txt[0].split()[0]
            if hot == "in":
                value = 2
            elif hot == "out":
                value =1
            elif hot == "move":
                value = 3
        elif self.ndev == 2:#m2
            try:
                with open(self.dir+"m2.txt","r") as f:
                    txt = f.readlines()
                txt = [txt[i].split()[0] for i in range(len(txt))]
                m2_move = txt[0]
                m2_dir = int(txt[1])
                m2_pos = float(txt[2])
                m2_param = float(txt[3])
            except:
                with open(self.dir+"m2.txt","r") as f:
                    txt = f.readlines()
                txt = [txt[i].split()[0] for i in range(len(txt))]
                m2_move = txt[0]
                m2_dir = txt[1]
                m2_pos = float(txt[2])
                m2_param = float(txt[3])
            print("m2_pos", m2_pos)
            real = abs(int(m2_pos/1000))
            decimal = abs(m2_pos/1000)-real
            _list = []
            for i in range(154):
                dt = i%16
                if not 0 <= dt <= 5 or i < 15:
                    _list.append(i)
            if no == 'FBIDIO_IN9_16':
                if m2_dir > 0:
                    value = real + 32
                elif m2_dir < 0:
                    value = real
            elif no == 'FBIDIO_IN1_8':
                value = _list[int(decimal*100)]
            
        else:
            with open(self.dir+"drive.txt","r") as f:
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
        with open(self.dir+"drive.txt","w") as f:
            f.write(str(self.drive))
            f.write("\n")
            f.write(str(self.contactor))
        return

    def drive_read(self):
        while True:
            try:
                with open(self.dir+"drive.txt","r") as df:
                    txt = df.readlines()
                    txt = [txt[i].split()[0] for i in range(len(txt))]
                    print(txt)
                    self.drive = int(txt[0])
                    self.contactor = int(txt[1])
                break
            except:
                pass
        return txt
