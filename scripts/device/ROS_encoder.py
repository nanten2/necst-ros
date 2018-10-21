#!/usr/bin/env python3

import math
import time
import sys
import threading
import rospy
import pyinterface
from datetime import datetime as dt

from necst.msg import Status_encoder_msg
from necst.srv import Bool_srv
from necst.srv import Bool_srvResponse

node_name = "encoder_status"

class enc_controller(object):

    Az = ''
    El = ''
    enc_Az = 0. #test
    enc_El = 45*3600. #test
    vel_az = 0.
    vel_el = 0.
    resolution = 360*3600/(23600*400)  #0.13728813559 (4 multiplication)

    origin_flag = False
    mode = ""
    
    def __init__(self):
        board_name = 6204
        rsw_id = 0
        self.dio = pyinterface.open(board_name, rsw_id)
        self.initialize()
        self.sub = rospy.Service("encoder_origin", Bool_srv, self.origin_setting)
        th = threading.Thread(target=self.origin_flag_check)
        th.start()
        pass

    def initialize(self):
        if self.dio.get_mode().to_bit() == "00000000" :
            self.dio.initialize()            
            self.board_setting()
        else:
            pass
        
    def origin_setting(self, req):
        if req.data == True:
            self.mode = "CLS0"
            self.origin_flag = True
            while self.origin_flag == True:
                time.sleep(0.1)
            return Bool_srvResponse(True)            
        else:
            self.mode = ""
            self.origin_flag = True
            while self.origin_flag == True:
                time.sleep(0.1)            
            return Bool_srvResponse(False)
        

    def origin_flag_check(self):
        while not rospy.is_shutdown():
            if self.origin_flag == True:
                self.board_setting(self.mode)
                self.origin_flag = False
            else:
                pass
            time.sleep(0.1)
        return
            
    def board_setting(self, z_mode=""):
        rospy.loginfo("initialize : start")
        self.dio.set_mode(mode="MD0 SEL1",direction=1, equal=0, latch=0, ch=1)
        self.dio.set_mode(mode="MD0 SEL1",direction=1, equal=0, latch=0, ch=2)
        self.dio.set_z_mode(clear_condition=z_mode, latch_condition="", z_polarity=0, ch=1)
        self.dio.set_z_mode(clear_condition=z_mode, latch_condition="", z_polarity=0, ch=2)
        print("origin setting mode : ", z_mode)        
        rospy.loginfo("initialize : end")
        return
        
    def pub_status(self):
        pub = rospy.Publisher("status_encoder", Status_encoder_msg, queue_size = 1, latch = True)
        msg = Status_encoder_msg()

        while not rospy.is_shutdown():
            #print("loop...")
            ret = self.get_azel()
            msg.enc_az = ret[0]
            msg.enc_el = ret[1]
            msg.from_node = node_name
            msg.timestamp = time.time()
            time.sleep(0.01)
            pub.publish(msg)
            rospy.loginfo('Az :'+str(msg.enc_az/3600.))
            rospy.loginfo('El :'+str(msg.enc_el/3600.))
        return

    def get_azel(self):
        cntAz = int(self.dio.get_counter(1).to_int())
        cntEl = int(self.dio.get_counter(2).to_int())
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
        encAz = cntAz*self.resolution
        self.Az = encAz      #arcsecond
        """ unsigned
        if cntEl < 360*3600./self.resolution:
            #encEl = (324*cntEl+295)/590
            encEl = cntEl*self.resolution
        else:
            encEl = -(2**32-cntEl)*self.resolution
            pass
        """
        encEl = cntEl*self.resolution
        self.El = encEl+45*3600      #arcsecond
            
        return [self.Az, self.El, _utc]


if __name__ == "__main__":
    rospy.init_node(node_name)
    enc = enc_controller()
    enc.pub_status()
    
