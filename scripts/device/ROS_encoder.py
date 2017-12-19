#!/usr/bin/env python3

import math
import time
import sys
import rospy
import pyinterface

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
    resolution = 360*3600/(23600*400)  #0.13728813559 (4 multiplication)

    def __init__(self):
        board_name = 6204
        rsw_id = 0
        rospy.init_node("encoder_status")
        sub = rospy.Subscriber("pyinterface", Status_encoder_msg, self.sub_enc)
        #sub = rospy.Subscriber("status_board", Test_board_msg, self.sub_enc)
        self.dio = pyinterface.open(board_name, rsw_id)
        self.board_initialize()
        pass

    def board_initialize(self):
        if self.dio.get_mode().to_bit() == "00000000" :
            rospy.loginfo("initialize : start")
            self.dio.initialize()
            self.dio.set_mode(mode="MD0 SEL1",direction=1, equal=0, latch=0, ch=1)
            self.dio.set_mode(mode="MD0 SEL1",direction=1, equal=0, latch=0, ch=2)
            self.dio.set_z_mode(clear_condition="CLS0", latch_condition="", z_polarity=0, ch=1)
            self.dio.set_z_mode(clear_condition="CLS0", latch_condition="", z_polarity=0, ch=2)
            rospy.loginfo("initialize : end")
        else:
            pass
            
    def pub_status(self):
        pub = rospy.Publisher("status_encoder", Status_encoder_msg, queue_size = 1, latch = True)
        msg = Status_encoder_msg()

        while not rospy.is_shutdown():
            print("loop...")
            ret = self.get_azel()
            #ret = self.test()
            msg.enc_az = ret[0]
            msg.enc_el = ret[1]
            time.sleep(0.01)
            pub.publish(msg)
            rospy.loginfo('Az :'+str(msg.enc_az/3600.))
            rospy.loginfo('El :'+str(msg.enc_el/3600.))
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
        cntAz = int(self.dio.get_counter(1).to_int())
        cntEl = int(self.dio.get_counter(2).to_int())
        print(cntAz)
        print(cntEl)
        if cntAz < 360*3600./self.resolution:
            #encAz = (324*cntAz+295)/590
            encAz = cntAz*self.resolution
        else:
            encAz = -(2**32-cntAz)*self.resolution
            pass
        self.Az = encAz      #arcsecond
            
        if cntEl < 360*3600./self.resolution:
            #encEl = (324*cntEl+295)/590
            encEl = cntEl*self.resolution
        else:
            encEl = -(2**32-cntEl)*self.resolution
            pass
        self.El = encEl+45*3600      #arcsecond
            
        return [self.Az, self.El]


if __name__ == "__main__":
    enc = enc_controller()
    enc.pub_status()
    
