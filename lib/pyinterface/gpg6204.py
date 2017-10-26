#!/usr/bin/env python
###this script is dummy for sumilator###
import time

class gpg6204(object):
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

    dome_enc = 0
    
    def __init__(self, ndev=1, remote=False):
        initialize = not remote
        self.ctrl = gpg6204_controller(ndev, initialize=initialize)
        pass

    def get_position(self):
        ret = self.ctrl.get_counter()
        return ret

    def di_check(self):
        ret = self.ctrl.input_di()
        
        return ret

    def do_output(self, ch, output_time=100):
        self.ctrl.output_do('OUT%d'%ch)
        if output_time==0: return
        time.sleep(output_time/1000.)
        self.ctrl.output_do(0)
        return

class gpg6204_controller(object):
    ndev = int()
    nChannel = int()
    boardid = ''
    print_log = True

    def __init__(self, ndev=1, nChannel=1, boardid=6204, initialize=True):
        self.ndev = ndev
        self.nChannel = nChannel
        return


    def get_counter(self):
        with open("/home/amigos/ros/src/necst/lib/"+"dome_enc.txt","r") as rf:
            txt = rf.readlines()
            txt = txt[0].split()[0]
        return float(txt)

    def input_di(self):
        return
    
    def output_do(self, ulValue):
        return
