#!/usr/bin/env python
import math
import time
#import portio
import rospy

from necst.msg import Status_encoder_msg
from necst.msg import Status_antenna_msg
from necst.msg import Test_board_msg

class enc_controller(object):

    Az = ''
    El = ''
    enc_Az = 0. #test
    enc_El = 45*3600. #test
    vel_az = 0.
    vel_el = 0.
    

    def __init__(self):
        rospy.init_node("encoder_status")
        #sub = rospy.Subscriber("pyinterface", Status_encoder_msg, self.sub_enc)
        sub = rospy.Subscriber("status_board", Test_board_msg, self.sub_enc)
        pass

    def pub_status(self):
        pub = rospy.Publisher("status_encoder", Status_encoder_msg, queue_size = 10, latch = True)
        msg = Status_encoder_msg()

        while not rospy.is_shutdown():
            print("loop...")
            #ret = self.get_azel()
            ret = self.test()
            msg.enc_az = ret[0]
            msg.enc_el = ret[1]
            time.sleep(0.1)
            pub.publish(msg)
            rospy.loginfo(msg.enc_az/3600.)
            rospy.loginfo(msg.enc_el/3600.)
        return

    def test(self):
        #self.Az = 40*3600.
        #self.El = 30*3600.
        self.enc_Az += self.vel_az * 0.1
        self.enc_El += self.vel_el * 0.1
        return [self.enc_Az, self.enc_El]

    def sub_enc(self, req):
        self.vel_az = req.enc_az
        self.vel_el = req.enc_el
        #print(req.enc_az, req.enc_el)
        return


    def get_azel(self):
        # for renishaw(El), for nikon(Az)
        byte_az = [0]*3
        
        #dioOutputByte(CONTROLER_BASE0,0x03,0x04);
        #CONRROLER_BASE0 = 0xc000
        portio.outb(0x04, (0x2050+0x03)) # Az
        
        time.sleep(3./1000) # need waiting
        #dioOutputByte(CONTROLER_BASE0,0x03,0x00);
        portio.outb(0x00, (0x2050+0x03))
        
        # get data from board
        for i in range(3):
            # byte_az[i] = dioInputByte(CONTROLER_BASE0,i);
            byte_az[i] = portio.inb((0x2050+i))
            
            # reverse byte
            # byte_az[i]=~byte_az[i];byte[2] is hugou 1keta+7keta suuji
            byte_az[i] = ~byte_az[i] & 0b11111111
            
        self.Az = self.bin2dec_2s_complement(byte_az, 3)
        
        #for loop test
        f = open("enc_test_log.txt","w")
        f.write("get_az")
        f.close()
        
        
        portio.outb(2, 0x2006)
        cntEl = portio.inl(0x2000)
        b_num = bin(cntEl)
        b_num = b_num.lstrip("0b")
        if len(b_num) == 32:
            if int(b_num[0]) == 1:
                b_num = int(b_num, 2)
                cntEl = -(~b_num & 0b01111111111111111111111111111111)
        if cntEl > 0:
            encEl = int((324.*cntEl+295.)/590.)
        else:
            encEl = int((324.*cntEl-295.)/590.)
        self.El = encEl+45.*3600.      #arcsecond
        
        #for loop test
        f = open("enc_test_log.txt","w")
        f.write("get_el")
        f.close()
        
        
                #print(self.Az/3600.)
        #print(self.El/3600.)
        return [self.Az, self.El]

    def bin2dec_2s_complement(self, byte, nSize):
        i = sign = ord = 1
        abs = 0
        if nSize == 0:
            return 0
        
        #sign = byte[nSize-1]>>7 ?-1:1;
        if byte[nSize-1]>>7 == True:
            sign = -1
        else:
            sign = 1
        """
        num = bin(byte[nSize-1])
        if len(num) == 11:
            num = num[3:10]
            byte[nSize-1] = int(num, 2)
        """
        if sign == 1:
            for i in range(nSize):
                abs += ord*byte[i]
                ord *= 256
            abs += ord*byte[nSize-1] & (~0x80 & 0b01111111)
            
        else:
            for i in range(nSize):
                abs += ord*(~byte[i] & 0b11111111)
                ord *= 256
            abs += 1
        
        return abs*sign

if __name__ == "__main__":
    enc = enc_controller()
    enc.pub_status()
    
