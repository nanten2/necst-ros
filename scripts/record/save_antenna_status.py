#!/usr/bin/env python3
import rospy
import sys
sys.path.append("/home/amigos/git")
import n2lite
import time
from necst.msg import Status_encoder_msg
from necst.msg import xffts_flag_msg
import threading
import queue

class telescope_logger():
    def __init__(self):
        self.path_to_db = "./otf_20190806_antenna_n31.db"
        self.regist_subscriber()
        ###parameter
        self.enc_az = 0
        self.enc_el = 0
        self.timestamp = 0
        self.queue = queue.Queue()
        self.obs_mode  = ""
        self.scan_num = ""
        ###flag
        self.__timestamp = 0
        self.newdb_name = "./otf_20190806_antenna_n31.db"
        self.before_dbname = ""
        self.config2()
        pass

    def regist_subscriber(self):
        rospy.Subscriber("status_encoder", Status_encoder_msg, self.callback, queue_size = 1)
        rospy.Subscriber("XFFTS_DB_flag", xffts_flag_msg, self.callback2, queue_size = 10)
        pass

    def callback(self, req):
        #self.enc_az = req.enc_az
        #self.enc_el = req.enc_el
        #self.timestamp = req.timestamp
        self.queue.put([req.enc_az, req.enc_el, req.timestamp])
        pass

    def callback2(self, req):
        self.__timestamp = req.timestamp
        self.newdb_name = "antenna"+req.newdb_name
        self.obs_mode = req.obs_mode
        self.scan_num = req.scan_num
        
    def config2(self):
        while self.newdb_name == self.before_dbname:
            time.sleep(0.1)#wait until queue has no items
            continue
        print("***NEW***", self.newdb_name)
        self.n = n2lite.xffts_logger("./{}".format(self.newdb_name))
        self.n.make_table("encoder", {"timestamp":"float", "enc_az":"float", "enc_el":"float", "obs_mode":"text", "scan_num":"text"})
        self.before_dbname = self.newdb_name
        print("new", self.newdb_name)###for debug

    def save_to_db(self):
        count = 0
        while not rospy.is_shutdown():
            ss = time.time()
            #if not self.__timestamp == 0:
            if True:
                tmp = self.queue.get()
                self.n.write("encoder", "", [tmp[2], tmp[0], tmp[1], self.obs_mode, self.scan_num], auto_commit = False)
                count += 1
                print("save {}".format(count))
            else:
                print("wait")
            if count > 1000:
                self.n.commit_data()
                count = 0
            time.sleep(0.01)

if __name__ == "__main__":
    rospy.init_node("sas")
    t = telescope_logger()
    s = threading.Thread(target = t.save_to_db)
    s.setDaemon(True)
    s.start()
    rospy.spin()
        
