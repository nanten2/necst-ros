import time
import sys

class gpg7204(object):
    def __init__(self, ndev=1, remote=False):
        initialize = not remote
        self.ctrl = gpg7204_controller(ndev, initialize=initialize)
        self.dir = "/home/necst/ros/src/necst/lib/"
        pass

    def stop(self):
        self.ctrl.stop_motion()
        return

    
    def move(self, speed, count, low_speed=5, acc=100, dec=100, sspeed=0):
        if speed >= 0: direc = 1
        else: direc = -1
        speed = abs(int(speed))
        count *= direc
        with open(self.dir+"m4.txt","r") as rf:
            txt = rf.readlines()
        txt = txt[0].split()[0]
        with open(self.dir+"m4.txt","w") as wf:
            if count > 0 and txt == "in":
                wf.write("move")
            elif count < 0 and txt == "out":
                wf.write("move")
            else:
                pass
        with open(self.dir+"m4.txt","w") as wf:
            if count > 0:
                wf.write("out")
            elif count < 0:
                wf.write("in")
            else:
                wf.write("move")
        return

    
    def get_position(self):
        ret = self.ctrl.get_counter()
        return ret
        


class gpg7204_controller(object):
    ndev = int()
    boardid = ''
    print_log = True
    
    def __init__(self, ndev=1, boardid=742020, initialize=True):
        """
        boardid = 7204 or 742020
        """
        self.ndev = ndev
        self.dir = "/home/necst/ros/src/necst/lib/"
        #if initialize: self.initialize()
        return

    def off_inter_lock(self):
        """
        4. MtrInterLock
        """
        return


    
    def set_motion(self, mode='MTR_JOG', motion_mode='MTR_ACC_NORMAL',low_speed=10, speed=100, acc=50, dec=50, sspeed=50, step=0):
        """
        10. MtrSetMotion
        ----------------
        """
        return

    def get_counter(self):
        """
        24. MtrReadCounter
        ------------------
        """
        with open(self.dir+"m4.txt","r") as rf:
            txt = rf.readlines()
        txt = txt[0].split()[0]
        return txt
        
    def get_status(self, mode='MTR_BUSY'):
        """
        22. MtrGetStatus
        ----------------
        """
        with open(self.dir+"m4.txt","r") as rf:
            txt = rf.readlines()
        txt = txt[0].split()[0]
        if txt == 'out':
            status = 4
        elif txt == 'in':
            status = 8
        elif txt == "move":
            status = 0
        return status

    def set_limit_config(self, mode='MTR_MASK', config=0):
        """
        7. MtrSetLimitConfig
        --------------------
        """
        return

    def stop_motion(self, mode='MTR_DEC_STOP'):
        with open(self.dir+"m4.txt","w") as wf:
            wf.write("move")
        return
