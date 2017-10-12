"""this script is dummy script for check
board.py(dummy)
"""
#import pyinterface

class board(object):
    
    def __init__(self):
        #self.dio = pyinterface.create_gpg2000(3)
        pass
        
    def out_word(self,no,value):
        #self.dio.ctrl.out_word(no,value)
        print('out_word pyinterface',no,value)
        
    def in_byte(self,no):
        #return self.dio.ctrl.in_byte(no)
        return 1
