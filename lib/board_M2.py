import pyinterface


class board(object):

    def __init__(self):
        self.dio = pyinterface.create_gpg2000(2)
        pass

    def in_byte(self, name):
        return self.dio.ctrl.in_byte(name)

    def out_byte(self, name, value):
        self.dio.ctrl.out_byte(name, value)
        return
