#import pyinterface

class board_drive(object):
    
    def __init__(self):
        #self.dio = pyinterface.create_gpg2000(4)
        pass
        
    def out_byte(self,no,value):
        if no == "FBIDIO_OUT1_8":
            if value == 3:
                #self.dio.ctrl.out_byte(no,value)
                print("drive_on")
            elif value == 0:
                #self.dio.ctrl.out_byte(no,value)
                print("drive_off")
            else:
                print("value error !!")
                return False
                
        elif no == "FBIDIO_OUT9_16":
            if value == 15:
                #self.dio.ctrl.out_byte(no,value)
                print("contactor_on")
            elif value == 0:
                #self.dio.ctrl.out_byte(no,value)
                print("contactor_off")
            else:
                #print("value error !!")
                return False
        else:
            #print("name error !!")
            return False

        return True

        
