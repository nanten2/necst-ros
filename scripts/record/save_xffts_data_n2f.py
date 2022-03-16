#!/usr/bin/env python3
import rospy
import sys
import time
import threading
import queue
import numpy
import n2df
from std_msgs.msg import String
from nascorx_xffts.msg import XFFTS_msg
from necst.msg import xffts_flag_msg


class xffts_logger:
    def __init__(self):
        self.queue = queue.Queue()
        ###register subscriber
        self.sub1 = rospy.Subscriber(
            "XFFTS_SPEC", XFFTS_msg, self.save_to_queue, queue_size=1000
        )
        self.sub2 = rospy.Subscriber(
            "XFFTS_DB_flag", xffts_flag_msg, self.update_flag, queue_size=1
        )
        ###parameter
        self.obs_mode = ""
        self.scan_num = 0
        self.lamdel = 0
        self.betdel = 0
        ###path
        self.path = ""
        pass

    def update_flag(self, req):
        if not req.newdb_name == "":
            if hasattr(self, "n"):
                self.n.close()
            self.n = n2df.File(req.newdb_name)
        self.obs_mode = req.obs_mode
        self.scan_num = req.scan_num
        self.lamdel = req.lamdel
        self.betdel = req.betdel
        self.path = req.newdb_name
        pass

    def save_to_queue(self, req):
        if not self.path == "":
            tmp_list = []
            for i in range(20):
                tmp_list += list(eval("req.SPEC_BE{}".format(i + 1)))
            self.queue.put(
                [
                    req.timestamp,
                    tmp_list,
                    str(self.obs_mode),
                    int(self.scan_num),
                    int(self.lamdel),
                    int(self.betdel),
                ]
            )
        else:
            pass

    def save_data(self):
        while not rospy.is_shutdown():
            if self.path == "":
                time.sleep(0.01)
                continue
            ret = self.queue.get()
            tmp_list = [float(ret[0]), [*ret[1]], ret[2], ret[3], ret[4], ret[5]]
            self.n.write(tmp_list)
            time.sleep(0.001)
            print("save")  # for debug will be deleted

    def pub_status(self):
        pub = rospy.Publisher("xffts_logger_status", String, queue_size=1)
        while not rospy.is_shutdown():
            pub.publish(
                "qsize : {}, datapath : {}".format(self.queue.qsize(), self.path)
            )  #
            time.sleep(0.5)


if __name__ == "__main__":
    rospy.init_node("xffts_logger")
    xffts = xffts_logger()
    s1 = threading.Thread(target=xffts.save_data)
    s1.setDaemon(True)
    s1.start()
    s2 = threading.Thread(target=xffts.pub_status)
    s2.setDaemon(True)
    s2.start()
    rospy.spin()
