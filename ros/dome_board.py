import pyinterface

class dome_board(object):
    def __init__(self):
        self.dio = pyinterface.create_gpg2000(5)
        pass

    def do_output(self, buff, a, b):
        self.dio.do_output(buff, a, b)
        return

    def di_check(self, a, b):
        self.dio.di_check(a, b)
        return
    
