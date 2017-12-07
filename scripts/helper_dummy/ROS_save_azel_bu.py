#!/usr/bin/env python
import rospy
import time
from necst.msg import Status_encoder_msg
from necst.msg import Status_antenna_msg


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
        
    def write_file(self):
        while not rospy.is_shutdown():
            ut = time.gmtime()
            tstmp = time.strftime("%Y_%m_%d", ut)
            ctime = time.time()
            f = open("./"+tstmp+".txt","a")
            f.write('%s %s %s %s %s \n'%(str(ctime), str(self.enc_az), str(self.enc_el), str(self.command_az), str(self.command_el)))
            f.close()
            time.sleep(0.1)
            continue
    
if __name__ == '__main__':
    st = save_azel()
    rospy.init_node('save_azel')
    ut = time.gmtime()
    print('start recording [filename :'+time.strftime("%Y_%m_%d", ut)+'.txt]')
    sub = rospy.Subscriber('status_encoder', Status_encoder_msg, st.callback)
    sub1 = rospy.Subscriber('status_antenna', Status_antenna_msg, st.callback2)
    st.write_file()
