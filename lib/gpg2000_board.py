#!/usr/bin/env python
import pyinterface

class board(object):

    def __init__(self):
        self.dio = pyinterface.create_gpg2000(3)
        pass

    def out_word(self,no,value):
        self.dio.ctrl.out_word(no,value)
        if no == "FBIDIO_OUT1_16":
            pass
        elif no == "FBIDIO_OUT17_32":
            pass
        else:
            pass
        return
            

    def in_byte(self,no):
        #return self.dio.ctrl.in_byte(no)
        return 


