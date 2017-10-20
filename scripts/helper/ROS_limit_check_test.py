# !usr/bin/env python

# test
import time
#import pyinterface
import rospy
from necst.msg import Status_limit_msg
from necst.msg import Status_encoder_msg
from necst.msg import Status_drive_msg


class limit(object):
    #self.dio = pyinterface.create_gpg2000(3)

    stop_flag = 0
    ret = [0]*4
    error_box = [1]*32
    msg = ""

    def sub_enc(self, req):
        self.error_box[0:1] = [req.value[0]]
        self.error_box[2:3] = [req.value[1]]


    def read(self):
        pub = rospy.Publisher("limit_check", Status_limit_msg, queue_size=10, latch=True)
        st = Status_limit_msg()
        while self.stop_flag == 0:
        
            try:
                st.error_box = self.error_box
                st.error_msg = self.msg
                pub.publish(st)
            except:
                rospy.logerr("no publish")
                self.stop_flag = 1

            time.slep(0.1)


if __name__ == "__main__":
    li = limit()
    rospy.init_node("limit_check")
    sub = rospy.Subscriber("status_drive", Status_drive_msg, li.sub_enc)
    time.sleep(3)
    li.read()
    
