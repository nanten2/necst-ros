#!/usr/bin/env python3

import rospy
import time
import os
import sys
from necst.msg import Status_encoder_msg
from necst.msg import Status_antenna_msg

# --
node_name = 'save_azel'
# --

class save_azel(object):
    enc_az = 0
    enc_el = 45*3600.
    enc_time = 0
    command_az = 0
    command_el = 45*3600.
    command_time = 0
    dir_name = ''
    flag = False
    
    def __init__(self):
        ut = time.gmtime()
        self.dir_name = time.strftime("/home/amigos/data/experiment/save_azel/%Y_%m_%d_%H_%M_%S", ut)
        os.mkdir(self.dir_name)
        pass

    def callback(self, req):
        self.enc_az = req.enc_az
        self.enc_el = req.enc_el
        self.enc_time = req.timestamp
        return
    
    def callback2(self, req):
        self.flag = True
        self.command_az = req.command_az
        self.command_el = req.command_el
        self.command_azspeed = req.command_azspeed
        self.command_elspeed = req.command_elspeed
        self.command_time = req.timestamp
        return
        
    def write_file(self):
        while not rospy.is_shutdown():
            #print('%13.2f %3.4f %3.4f %13.2f %3.4f %3.4f'%(self.enc_time., self.enc_az/3600., self.enc_el/3600., self.command_time., self.command_az/3600., self.command_el/3600.))
            while(self.enc_time==0 or self.command_time==0 or self.flag==False):
                print('waiting subscribe')
            saveto1 = os.path.join(self.dir_name, 'enc.txt')
            saveto2 = os.path.join(self.dir_name, 'command.txt')
            saveto3 = os.path.join(self.dir_name, 'command_speed.txt')
            print('save data!!')
            f1 = open(saveto1,'a')
            f2 = open(saveto2,'a')
            f3 = open(saveto3,'a')
            f1.write('%s %s %s \n'%(str(self.enc_time), str(self.enc_az), str(self.enc_el)))
            f1.close
            f2.write('%s %s %s \n'%(str(self.command_time), str(self.command_az), str(self.command_el)))
            f2.close()
            f3.write('%s %s %s \n'%(str(self.command_time), str(self.command_azspeed), str(self.command_elspeed)))
            f3.close()
            time.sleep(0.01)
            continue
        return
    
if __name__ == '__main__':
    st = save_azel()
    rospy.init_node(node_name)
    ut = time.gmtime()
    print('start recording')
    sub = rospy.Subscriber('status_encoder', Status_encoder_msg, st.callback, queue_size=1)
    sub2 = rospy.Subscriber('status_antenna', Status_antenna_msg, st.callback2, queue_size=1)
    st.write_file()
