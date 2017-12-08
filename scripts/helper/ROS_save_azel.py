#!/usr/bin/env python3

import rospy
import time
import os
from necst.msg import Status_encoder_msg
from necst.msg import Status_antenna_msg

# --
data_exp_dir = '/home/amigos/data/experiment'
node_name = 'save_azel'
home_dir = os.path.join(data_exp_dir, node_name)
# --

class save_azel(object):
    enc_az = 0
    enc_el = 45*3600.
    command_az = 0
    command_el = 45*3600.

    def __init__(self):
        pass

    def callback(self, req):
        self.enc_az = req.enc_az
        self.enc_el = req.enc_el
        return
    
    def callback2(self, req):
        self.command_az = req.command_az
        self.command_el = req.command_el
        return
        
    def write_file(self):
        ut = time.gmtime()
        filename = time.strftime("%Y_%m_%d_%H_%M_%S.txt", ut)
        saveto = os.path.join(home_dir, filename)
        while not rospy.is_shutdown():
            ctime = time.time()
            print('%13.2f %3.4f %3.4f %3.4f %3.4f'%(ctime, self.enc_az/3600., self.enc_el/3600., self.command_az/3600., self.command_el/3600.))
            f = open(saveto,'a')
            f.write('%s %s %s %s %s \n'%(str(ctime), str(self.enc_az), str(self.enc_el), str(self.command_az), str(self.command_el)))
            f.close()
            time.sleep(0.1)
            continue
        return
    
if __name__ == '__main__':
    if not os.path.exists(home_dir):
        os.makedirs(home_dir)
        pass
    
    st = save_azel()
    rospy.init_node(node_name)
    ut = time.gmtime()
    print('start recording [filename :'+time.strftime("%Y_%m_%d_%H_%M_%S", ut)+'.txt]')
    sub = rospy.Subscriber('status_encoder', Status_encoder_msg, st.callback)
    sub1 = rospy.Subscriber('status_antenna', Status_antenna_msg, st.callback2)
    st.write_file()
