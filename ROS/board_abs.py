import pyinterface

class board_abs(object):

    def __init__(self):
        self.dio = pyinterface.create_gpg2000(3)

    def out_byte(self, no, value):
        self.dio.ctrl.out_byte(no,value)

    def in_byte(self,no):
        return self.dio.ctrl.in_byte(no)
