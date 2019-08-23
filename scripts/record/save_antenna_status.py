#!/usr/bin/env python3
import rospy
import sys
import n2lite
import time
from necst.msg import Status_encoder_msg
from necst.msg import xffts_flag_msg
from necst.msg import encdb_flag_msg
import threading
import queue

class telescope_logger():
    def __init__(self):
        self.path_to_db = "./test.db"
        self.regist_subscriber()
        # parameter
        # =========
        self.enc_az = 0
        self.enc_el = 0
        self.timestamp = 0
        self.queue = queue.Queue()
        self.obs_mode = ""
        self.scan_num = ""
        # flag
        # ====
        self.boolflag = False
        self.newdb_name = ""
        self.before_dbname = ""
        self.flag = False
        pass

    def regist_subscriber(self):
        rospy.Subscriber("status_encoder", Status_encoder_msg, self.callback, queue_size=1)
        rospy.Subscriber("encoder_DB_flag", encdb_flag_msg, self.callback2, queue_size=1)
        pass

    def callback(self, req):
        if self.boolflag:
            self.queue.put([req.enc_az, req.enc_el, req.timestamp])
            print("queue")
        else:
            pass

    def callback2(self, req):
        self.boolflag = req.data
        self.newdb_name = req.newdb_name
        print(self.newdb_name)

    def main(self):
        while not rospy.is_shutdown():
            while self.newdb_name == self.before_dbname :
                time.sleep(0.5)
                print("wait") # for debug
            self.config()
            self.flag = True
        
    def config(self):
        print("***NEW***", self.newdb_name)
        self.n = n2lite.xffts_logger(self.newdb_name)
        self.n.make_table("encoder", {"timestamp": "float", "enc_az": "float",
                                      "enc_el": "float", "obs_mode": "text", "scan_num": "text"})
        self.before_dbname = self.newdb_name
        print("new", self.newdb_name) # for debug

    def save_to_db(self):
        count = 0
        while not rospy.is_shutdown():
            ss = time.time()
            if not self.flag:
                continue
            if self.boolflag:
                tmp = self.queue.get()
                self.n.write("encoder", "", [tmp[2], tmp[0], tmp[1], self.obs_mode,
                                             self.scan_num], auto_commit=False)
                count += 1
                print("save {}".format(count))
            elif self.queue.qsize() > 0:
                tmp = self.queue.get()
                self.n.write("encoder", "", [tmp[2], tmp[0], tmp[1], self.obs_mode,
                                             self.scan_num], auto_commit=False)
                count += 1
                print("save2 {}".format(count))
                self.n.commit_data()
            else:
                print("wait")
            if count > 100:
                self.n.commit_data()
                count = 0
            time.sleep(0.01)

if __name__ == "__main__":
    rospy.init_node("sas")
    t = telescope_logger()
    s1 = threading.Thread(target=t.save_to_db)
    s1.setDaemon(True)
    s1.start()
    s2 = threading.Thread(target=t.main)
    s2.setDaemon(True)
    s2.start()
    rospy.spin()
