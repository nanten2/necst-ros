#!/usr/bin/env python3

import math
import time
import sys
import rospy
#import pyinterface

from necst.msg import Status_encoder_msg
from necst.msg import Status_antenna_msg
from necst.srv import Bool_srv
from necst.srv import Bool_srvResponse

from datetime import datetime as dt

node_name = "encoder_status"
sys.path.append("/home/amigos/ros/src/necst/lib")
import topic_status
deco = topic_status.deco(node_name)

class enc_controller(object):

    Az = ''
    El = ''
    enc_Az = 0. #test
    enc_El = 45*3600. #test
    vel_az = 0.
    vel_el = 0.
    resolution = 360*3600/(23600*400)  #0.13728813559 (4 multiplication)

    def __init__(self):
        #board_name = 6204
        #rsw_id = 0
        sub = rospy.Subscriber("pyinterface", Status_encoder_msg, self.sub_enc)
        sub2 = rospy.Subscriber('status_antenna', Status_antenna_msg, self.sub_antenna, queue_size=1)
        #self.dio = pyinterface.open(board_name, rsw_id)
        self.initialize()
        self.sub = rospy.Service("encoder_origin", Bool_srv, self.origin_setting)
        pass

    def initialize(self):
        if True:#self.dio.get_mode().to_bit() == "00000000" :
            #self.dio.initialize()            
            self.board_setting()
        else:
            pass
        
    def origin_setting(self, req):
        print("origin setting mode : ", req.data)
        if req.data == True:
            self.board_setting("CLS0")
            return Bool_srvResponse(True)            
        else:
            self.board_setting()
            return Bool_srvResponse(False)

    def board_setting(self, z_mode=""):
        rospy.loginfo("setting : start")
        #self.dio.set_mode(mode="MD0 SEL1",direction=1, equal=0, latch=0, ch=1)
        #self.dio.set_mode(mode="MD0 SEL1",direction=1, equal=0, latch=0, ch=2)
        #self.dio.set_z_mode(clear_condition=z_mode, latch_condition="", z_polarity=0, ch=1)
        #self.dio.set_z_mode(clear_condition=z_mode, latch_condition="", z_polarity=0, ch=2)
        print("z_mode is : ", z_mode)
        rospy.loginfo("initialize : end")
        return

    def sub_antenna(self, req):
        print('get status antenns')
        print("speed", req.command_azspeed)
        self.vel_az = req.command_azspeed
        self.vel_el = req.command_elspeed
        self.enc_Az += self.vel_az*0.01*0.617#0.617 = 1/1.6 real_vel/command_vel=1.6
        self.enc_El += self.vel_el*0.01*0.617#
        return

    @deco
    def pub_status(self):
        pub = rospy.Publisher("status_encoder", Status_encoder_msg, queue_size = 1, latch = True)
        msg = Status_encoder_msg()

        while not rospy.is_shutdown():
            ##print("loop...")
            #ret = self.get_azel()
            #ret = self.test()
            
            msg.enc_az = self.enc_Az
            msg.enc_el = self.enc_El
            msg.from_node = node_name
            msg.timestamp = time.time()
            time.sleep(0.01)
            pub.publish(msg)
            rospy.loginfo('Az :'+str(msg.enc_az/3600.))
            rospy.loginfo('El :'+str(msg.enc_el/3600.))
        return

    def test(self):
        #self.Az = 40*3600.
        #self.El = 30*3600.
        self.enc_Az += self.vel_az * 0.01
        self.enc_El += self.vel_el * 0.01
        return [self.enc_Az, self.enc_El]

    def sub_enc(self, req):
        self.vel_az = req.enc_az
        self.vel_el = req.enc_el
        #print(req.enc_az, req.enc_el)
        return

    '''
    def get_azel(self):
        #cntAz = int(self.dio.get_counter(1).to_int())
        #cntEl = int(self.dio.get_counter(2).to_int())
        now = dt.utcnow()
        _utc = now.strftime("%Y-%m-%d %H:%M:%S")
        #print(cntAz)
        #print(cntEl)
        
        """unsigned
        if cntAz < 360*3600./self.resolution:
            #encAz = (324*cntAz+295)/590
            encAz = cntAz*self.resolution
        else:
            encAz = -(2**32-cntAz)*self.resolution
            pass
        """
        #encAz = cntAz*self.resolution
        #self.Az = encAz      #arcsecond
        """ unsigned
        if cntEl < 360*3600./self.resolution:
            #encEl = (324*cntEl+295)/590
            encEl = cntEl*self.resolution
        else:
            encEl = -(2**32-cntEl)*self.resolution
            pass
        """
        #encEl = cntEl*self.resolution
        #self.El = encEl+45*3600      #arcsecond
            
        #return [self.Az, self.El, _utc]
    '''

if __name__ == "__main__":
    rospy.init_node(node_name)
    enc = enc_controller()
    enc.pub_status()
    
