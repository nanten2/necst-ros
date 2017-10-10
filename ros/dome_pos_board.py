import pyinterface

class dome_pos_board(object):

    def __init__(self, ndev2 = 1):
        self.dio = pyinterface.create_gpg6204(ndev2)

    def ctrl_reset(self):
        self.dio.ctrl_reset()
        return

    def ctrl_set_mode(self, a, b, c, d):
        self.dio.ctrl_set_mode(a, b, c, d)
        return

    def get_position(self):
        return self.dio.get_position()

    def set_counter(self, counter):
        self.dio.ctrl.set_counter(counter)
        
        
