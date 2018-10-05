#!/usr/bin/env python3

import rospy
import time
import os
import sys
import datetime
from necst.msg import Status_encoder_msg
from necst.msg import Status_pid_msg

# --
node_name = 'log_pid'
# --

class save_pid(object):
    rate_az = 0
    rate_el = 0
    target_arcsec_az = 0
    target_arcsec_el = 0
    enc_arcsec_az = 0
    enc_arcsec_el = 0
    pre_hensa_az = 0
    pre_hensa_el = 0
    ihensa_az = 0
    ihensa_el = 0
    enc_before_az = 0
    enc_before_el = 0
    p_az = 0
    p_el = 0
    i_az = 0
    i_el = 0
    d_az = 0
    d_el = 0
    t_now = 0
    t_past = 0

    
    def __init__(self):
        ut = time.gmtime()
        pass

    def callback(self, req):
        self.rate_az = req.rate_az
        self.p_az = req.p_az
        self.i_az = req.i_az
        self.d_az = req.d_az
        self.target_arcsec_az = req.target_arcsec_az
        self.enc_arcsec_az = req.enc_arcsec_az
        self.pre_hensa_az = req.pre_hensa_az
        self.ihensa_az = req.ihensa_az
        self.enc_before_az = req.enc_before_az
        self.rate_el = req.rate_el
        self.p_el = req.p_el
        self.i_el = req.i_el
        self.d_el = req.d_el
        self.target_arcsec_el = req.target_arcsec_el
        self.enc_arcsec_el = req.enc_arcsec_el
        self.pre_hensa_el = req.pre_hensa_el
        self.ihensa_el = req.ihensa_el
        self.enc_before_el = req.enc_before_el
        self.t_now = req.t_now
        self.t_past = req.t_past
        return
           
    def write_file(self):
        while not rospy.is_shutdown():
            while(self.t_now==0):
                print('waiting subscribe')
            now = datetime.datetime.now()
            dir_name = '/home/amigos/log/{}/{}/{}'.format(now.year, now.month, now.day)
            
            saveto1 = os.path.join(dir_name, 'pid_param.txt')


            print('save data!!')
            f1 = open(saveto1,'a')

            f1.write('%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n'%(str(self.rate_az), str(self.p_az), str(self.i_az), str(self.d_az), str(self.target_arcsec_az), str(self.enc_arcsec_az), str(self.pre_hensa_az), str(self.ihensa_az), str(self.enc_before_az), str(self.rate_el), str(self.p_el), str(self.i_el), str(self.d_el), str(self.target_arcsec_el), str(self.enc_arcsec_el), str(self.pre_hensa_el), str(self.ihensa_el), str(self.enc_before_el),  str(self.t_now), str(self.t_past)))
            f1.close()
            time.sleep(1)
            continue
        return
    
if __name__ == '__main__':
    sp = save_pid()
    rospy.init_node(node_name)
    ut = time.gmtime()
    print('start recording')
    sub = rospy.Subscriber('status_pid', Status_pid_msg, sp.callback, queue_size=1)
    sp.write_file()
