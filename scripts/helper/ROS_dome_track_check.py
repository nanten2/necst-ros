#!/usr/bin/env python3
import time
import threading
import rospy
from necst.msg import Status_dome_msg
from necst.msg import Status_encoder_msg
from necst.msg import Bool_necst

node_name = 'dome_tracking'

class dome_tracking_check(object):
    enc_param = {
        'enc_az' : 0,
        'enc_el' : 0
        }

    dome_az = '0'
    tracking = False

    def __init__(self):
        self.start_thread()
        pass

    def start_thread(self):
        th = threading.Thread(target = self.pub_tracking)
        th.setDaemon(True)
        th.start()
        

    def set_dome_param(self, req):
        self.dome_az = req.dome_enc#[arcsec]
        return

    def set_enc_param(self, req):
        self.enc_param['enc_az'] = req.enc_az
        self.enc_param['enc_el'] = req.enc_el
        return

    def check_dome_track(self):
        dome_az = float(self.dome_az)
        enc_az = float(self.enc_param['enc_az'])
        """
        if dome_az < 0:
            dome_az += 360*3600
        if enc_az <0:
            enc_az += 360*3600
        """
        d_az = abs(dome_az - enc_az)
        #rospy.logwarn(d_az)

        if d_az <= 4*3600:
            self.tracking = True
        else:
            self.tracking = False
        time.sleep(0.2)
        rospy.loginfo('tracking : %s'%self.tracking)

    def pub_tracking(self):
        pub = rospy.Publisher('dome_tracking_check', Bool_necst, queue_size = 10, latch = True)
        track_status = Bool_necst()
        while not rospy.is_shutdown():
            self.check_dome_track()
            if self.tracking:
                track_status.data = True
            else:
                track_status.data = False
                pass
            track_status.from_node = node_name
            track_status.timestamp = time.time()
            pub.publish(track_status)
            time.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node(node_name)
    dc = dome_tracking_check()
    sub1 = rospy.Subscriber('status_dome', Status_dome_msg, dc.set_dome_param)
    sub2 = rospy.Subscriber('status_encoder',Status_encoder_msg, dc.set_enc_param)
    rospy.spin()
