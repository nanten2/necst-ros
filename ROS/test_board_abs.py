import rospy
import random
#import pyinterface



class board(object):
    #_pro = 0
    pos = 'OUT'

    def __init__(self):
        #self.dio = pyinterface.create_gpg2000(3)
        #rospy.init_node("board")
        pass

    def out_byte(self, no, value):
        buff = 0x01
        if value == buff :
            self.pos = 'IN'
        else :
            self.pos = 'OUT'
        print('POS : {0}'.format(self.pos))
        return
        
      

    def in_byte(self,no):
        #return self.dio.ctrl.in_byte(no)
        #num = random.choice([0x01, 0x02, 0x03])
        if self.pos == 'IN':
            num = 0x02
        elif self.pos == 'OUT':
            num = 0x01
        elif self.pos == 'MOVE':
            num = 0x03
        return num
 
        

