class gpg7204(object):
    def __init__(self, ndev=1, remote=False):
        initialize = not remote
        self.ctrl = gpg7204_controller(ndev, initialize=initialize)
        pass
            
    def start(self, speed, low_speed=5, acc=100, dec=100, sspeed=0):
        if speed >= 0: direc = 1
        else: direc = -1
        speed = abs(int(speed))
        self.ctrl.set_motion('MTR_JOG', 'MTR_ACC_NORMAL', low_speed=low_speed, speed=speed, acc=acc, 
                             dec=dec, sspeed=sspeed, step=direc)
        self.ctrl.start_motion('MTR_JOG')
        return
    
    def stop(self):
        self.ctrl.stop_motion()
        return
        
    def change_speed(self, new_speed, mode='MTR_ACCDEC_CHANGE'):
        new_speed = abs(new_speed)
        self.ctrl.change_speed(mode, new_speed)
        return
    
    def move(self, speed, count, low_speed=5, acc=100, dec=100, sspeed=0):
        if speed >= 0: direc = 1
        else: direc = -1
        speed = abs(int(speed))
        count *= direc
        self.ctrl.set_motion('MTR_PTP', 'MTR_ACC_NORMAL', low_speed=low_speed, speed=speed, acc=acc, dec=dec, sspeed=sspeed, step=count)
        self.ctrl.start_motion('MTR_PTP')
        return
        
    def move_with_lock(self, speed, count, low_speed=5, acc=100, dec=100, sspeed=0):
        initial_position = self.ctrl.get_counter()
        self.move(speed, count, low_speed=5, acc=100, dec=100, sspeed=0)
        while True:
            self.ctrl.print_log = False
            current_speed = self.ctrl.get_speed()
            if current_speed == 0: break
            position = abs(initial_position - self.ctrl.get_counter())
            sys.stdout.write('\rmoving... %5d/%d %d'%(position, abs(count), current_speed))
            sys.stdout.flush()
            time.sleep(0.1)
            continue
        self.ctrl.print_log = True
        print('')
        return
        
    def move_org(self, speed=1000, low_speed=5, acc=100, dec=100, sspeed=0):
        initial_position = self.ctrl.get_counter()
        speed = abs(int(speed))
        step = initial_position * -1
        self.move_with_lock(speed, step, low_speed=low_speed, acc=acc, dec=dec, sspeed=sspeed)
        return
    
    def set_org(self):
        self.ctrl.clear_counter()
        return
        
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


class gpg7204_controller(object):
    ndev = int()
    boardid = ''
    print_log = True
    
    def __init__(self, ndev=1, boardid=742020, initialize=True):
        """
        boardid = 7204 or 742020
        """
        self.ndev = ndev
        self.boardid = BoardID.verify(boardid)
        if initialize: self.initialize()
        return
