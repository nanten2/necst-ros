#!/usr/bin/env python
import pyinterface

class board(object):

    def __init__(self):
        self.dio = pyinterface.create_gpg2000(3)
        self.dio2 = pyinterface.create_gpg2000(4)
        pass

    def out_word(self,no,value):
        self.dio.ctrl.out_word(no,value)
        return
            
    def in_byte(self,no):
        return self.dio.ctrl.in_byte(no)


    def out_byte(self,no,value):
        if no == "FBIDIO_OUT1_8":
            if value == 3:
                self.dio2.ctrl.out_byte(no,value)
                print("contactor_on")
            elif value == 0:
                self.dio2.ctrl.out_byte(no,value)
                print("contactor_off")
            else:
                print("value error !!")
                return False
        elif no == "FBIDIO_OUT9_16":
            if value == 15:
                self.dio2.ctrl.out_byte(no,value)
                print("contactor_on")
            elif value == 0:
                self.dio2.ctrl.out_byte(no,value)
                print("contactor_off")
            else:
                print("value error !!")
                return False
        else:
            print("name error !!")
            return False

        return True
 
                


