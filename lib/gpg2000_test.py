import pyinterface

class board(object):

    def __init__(self):
        #self.dio = pyinterface.create_gpg2000(3)
        self.dio = pyinterface.create_gpg2000(10)#test
        pass

    def out_byte(self, no, value):
        self.dio.ctrl.out_byte(no,value)
        return

    def in_byte(self,no):
        return self.dio.ctrl.in_byte(no)
