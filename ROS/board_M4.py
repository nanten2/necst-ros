import pyinterface

class board_M4(object):
    
    def __init__(self):
        self.mtr = pyinterface.create_gpg7204(1)

    def get_position(self):
	self.mtr.get_position()

    def off_inter_lock(self):
        self.mtr.ctrl.off_inter_lock()

    def set_limit_config(self, mode, config):
	self.mtr.ctrl.set_limit_config(mode, config)

    def get_status(self, mode):
	self.mtr.ctrl.get_status(mode)

    def move(self, speed, count, low_speed=5, acc=100, dec=100, sspeed=0)
	self.mtr..move(speed, count, low_speed, acc, dec, sspeed=0)
