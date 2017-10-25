import rospy
#import pyinterface

class board(object):
    pos = 'IN'
    
    def __init__(self):
        pass

    def get_position(self):
        pass

    def off_inter_lock(self):
        pass

    def set_limit_config(self, mode, config):
        pass

    def get_status(self, mode):
        if self.pos == 'IN':
            num = 0x0008
        elif self.pos == 'OUT':
            num = 0x0004
        elif self.pos == 'MOVE':
            num = 0x0000
        return num

    def move(self, speed, nstep, low_speed=5, acc=100, dec=100, sspeed=0):
        if nstep == -60500 :
            self.pos = 'IN'
        elif nstep == 60500 :
            self.pos = 'OUT'
        print('POS : {0}'.format(self.pos))
        return

