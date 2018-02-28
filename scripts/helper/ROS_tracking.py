#!/usr/bin/env python3

import time
import threading
import rospy
from necst.msg import Status_antenna_msg
from necst.msg import Status_encoder_msg
from std_msgs.msg import Bool

class tracking_check(object):
    enc_param = {
        'enc_az' : 0,
        'enc_el' : 0
        }
    antenna_param = {
        'command_az' : 200,
        'command_el' : 200
        }
    tracking = False

    def __init__(self):
        rospy.init_node('tracking')
        self.start_thread()
        pass

    def start_thread(self):
        th = threading.Thread(target = self.pub_tracking)
        th.setDaemon(True)
        th.start()
        check = threading.Thread(target = self.check_track)
        check.setDaemon(True)
        check.start()        
        

    def set_ant_param(self, req):
        #rospy.loginfo(req.command_az)
        #rospy.loginfo(req.command_el)
        self.antenna_param['command_az'] = req.command_az
        self.antenna_param['command_el'] = req.command_el

    def set_enc_param(self, req):
        #rospy.loginfo(req.enc_az)
        #rospy.loginfo(req.enc_el)
        self.enc_param['enc_az'] = req.enc_az
        self.enc_param['enc_el'] = req.enc_el

    def check_track(self):
        track_count = 0
        while not rospy.is_shutdown():
            command_az = self.antenna_param['command_az']
            command_el = self.antenna_param['command_el']
            enc_az = self.enc_param['enc_az']
            enc_el = self.enc_param['enc_el']
            if command_az < 0:
                command_az += 360*3600
            if enc_az <0:
                enc_az += 360*3600
            d_az = abs(command_az - enc_az)
            d_el = abs(command_el - enc_el)
            #rospy.loginfo( self.antenna_param['command_az'])
            if d_az <= 3 and d_el <=3:
                track_count += 1
            else:
                track_count = 0
            if track_count >= 3:
                self.tracking = True
                track_count = 3
            else:
                self.tracking = False
            time.sleep(0.1)
            rospy.loginfo('tracking : %s'%self.tracking)
        return self.tracking

    def pub_tracking(self):
        pub = rospy.Publisher('tracking_check', Bool, queue_size = 10, latch = True)
        track_status = Bool()
        while not rospy.is_shutdown():
            if self.tracking:
                track_status.data = True
            else:
                track_status.data = False
            pub.publish(track_status)
            time.sleep(0.01)

if __name__ == '__main__':
    t = tracking_check()
    sub1 = rospy.Subscriber('status_antenna', Status_antenna_msg, t.set_ant_param)
    sub2 = rospy.Subscriber('status_encoder',Status_encoder_msg, t.set_enc_param)
    rospy.spin()
