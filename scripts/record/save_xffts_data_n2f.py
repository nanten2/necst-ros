#!/usr/bin/env python
import rospy
import sys
import time
import threading
import queue
import numpy
import n2f
from std_msgs.msg import String
from nascorx_xffts.msg import XFFTS_msg
from necst.msg import xffts_flag_msg

class xffts_logger():
    def __init__(self):
        self.path_to = ""
        self.queue = queue.Queue()
        ###register subscriber
        self.sub1 = rospy.Subscriber("XFFTS_SPEC", XFFTS_msg, self.save_to_queue, queue_size = 1000)
        self.sub2 = rospy.Subscriber("XFFTS_DB_flag", xffts_flag_msg, self.update_flag, queue_size = 1)
        ###ctrl flag
        self.timestamp = 0
        ###parameter
        self.obs_mode = ""
        self.scan_num = 0
        self.lamdel = 0
        self.betdel = 0
        pass


    def update_flag(self, req):
        self.timestamp = req.timestamp
        self.obs_mode = req.obs_mode
        self.scan_num = req.scan_num
        self.lamdel = req.lamdel
        self.betdel = req.betdel
        print("check update flag")
        pass
    
    def save_to_queue(self, req):
        if not self.timestamp == 0:
            tmp_list = [0]*20
            for i in range(20):
                tmp_list[i] = list(eval("req.SPEC_BE{}".format(i+1)))
            #tmp_list = [list(eval("req.SPEC_BE{}".format(i+1))) for i in range(20)]
            self.queue.put([req.timestamp, tmp_list])
        else:
            pass
        
    def save_data(self):
        path = "/home/amigos/ros/src/necst/scripts/record/hdd/test01.dat"#tmp
        n = n2f.File(path)#tmp
        while not rospy.is_shutdown():
            ret = self.queue.get()
            tmp_list = [float(ret[0]), ret[1][0], str(self.obs_mode), int(self.scan_num), int(self.lamdel), int(self.betdel)]
            print(numpy.shape(tmp_list[1]))
            n.write(tmp_list)
            time.sleep(0.001)
            print("save")
            

    def pub_status(self):
        pub = rospy.Publisher("logger_status", String, queue_size = 1)
        while not rospy.is_shutdown():
            pub.publish(str(self.queue.qsize()))
            time.sleep(1)
            


if __name__ == "__main__":
    rospy.init_node("xffts_logger")
    xffts = xffts_logger()
    s1 = threading.Thread(target = xffts.save_data)
    s1.setDaemon(True)
    s1.start()
    s2 = threading.Thread(target = xffts.pub_status)
    s2.setDaemon(True)
    s2.start()
    rospy.spin()
