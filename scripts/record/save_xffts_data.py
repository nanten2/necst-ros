#!/usr/bin/env python3
import rospy
import pickle
import sys
sys.path.append("/home/amigos/git")
import n2lites as n2lite
#from nascorx_xffts.msg import XFFTS_pm_msg
#from nascorx_xffts.msg import XFFTS_temp_msg_du
import time
import threading
import queue
#import tracemalloc
import gc

class xffts_logger():
    def __init__(self, table_name):
        self.path_to_db = "./test.db"
        self.table_name = table_name
        self.config()
        self._output_status()
        self.count = 0
        self.count1 = 0
        ###Queue
        self.queue = queue.Queue()
        self.queue2 = queue.Queue()
        ###status flag
        self.hozon_now = False
        self.timestamp = 0
        self.newdb_name = ""
        self.before_dbname = ""
        self.status = 0#0(first)/1(queue>30)/2(stop save queue>0)/3(else)
        ##param
        self.obs_mode = ""
        self.scan_num = ""
        ###regist subscriber
        self.regist_subscriber()
        print("ready")
        pass

    def main(self):
        while not rospy.is_shutdown():
            if self.timestamp != 0 and self.newdb_name != self.before_dbname and self.status == 3:
                self.config2()
            time.sleep(0.1)

    def config(self):
        self.n = n2lite.xffts_logger(self.path_to_db)
        self.n.make_table(self.table_name, {"timestamp":"float", "spec_array":"blob"})

    #def return_queuesize():
    #    return self.queue.qsize()

    def config2(self):
        while self.queue.qsize() > 0 and self.status == 3 and self.newdb_name != self.before_dbname:
            time.sleep(0.1)#wait until queue has no items
            continue
        print("***NEW***", self.newdb_name)
        self.n = n2lite.xffts_logger("./{}".format(self.newdb_name))
        self.n.make_table(self.table_name, {"timestamp":"float", "spec_array":"blob"})
        self.before_dbname = self.newdb_name
        print("new", self.newdb_name)###for debug

    def flag(self, req):
        self.timestamp = req.timestamp
        self.newdb_name = req.newdb_name
        self.obs_mode = req.obs_mode
        self.scan_num = req.scan_num
        print(req)
        
    def _output_status(self):
        print_str = f"DB path = {self.path_to_db}\n ***start logging***"
        print(print_str)

    def regist_subscriber(self):
        from nascorx_xffts.msg import XFFTS_msg
        from necst.msg import xffts_flag_msg
        self.sub1 = rospy.Subscriber("XFFTS_SPEC", XFFTS_msg, self.save_to_queue, queue_size = 100)
        self.sub2 = rospy.Subscriber("XFFTS_DB_flag", xffts_flag_msg, self.flag, queue_size = 10)

    def save_to_queue(self, req):
        if not self.timestamp == 0:
            tmplist = [None]*20
            start = time.time()
            for i in range(20):
                tmplist[i] = list(eval("req.SPEC_BE{}".format(i+1)))
            print("check : ",time.time()-start)
            print(type(tmplist[0]))
            print(type(tmplist[0][0]))
            self.queue.put(tmplist)
            self.queue2.put([self.obs_mode, self.scan_num])
            self.count1+=1
        else:
            pass
    
    def save_to_DB(self, req):
        array_pickle1 = pickle.dumps(req)
        self.n.write_blob(self.table_name, [time.time(), array_pickle1], auto_commit = True)
        self.count +=1
        print(self.count, __file__, "test comment")

    def save_to_db_tmp(self):
        while True:
            if self.queue.qsize() > 30:
                self.status = 1
                start = time.time()
                tmp_list = [None]*30
                tmp_list2 = [None]*30
                print("time check1###", time.time()-start)
                time1 = time.time()
                for i in range(30):
                    tmp_b  = self.queue2.get()
                    tmp_list[i] = ((time.time(),pickle.dumps(self.queue.get()), tmp_b[0], tmp_b[1]))
                    self.count+=1
                print("time check2###", time.time()-time1)
                time2 = time.time()
                self.n.write_blob2(self.table_name, tmp_list, auto_commit = False)
                tc1 = time.time()-time2
                time3 = time.time()
                self.n.commit_data()
                tc2 = time.time()-time3
                print("insert : {}  commit {}".format(tc1, tc2))
            elif self.queue.qsize() > 0 and self.timestamp == 0:
                self.status = 2
                tmp_list = [None]*self.queue.qsize()
                for i in range(self.queue.qsize()):
                      tmp_list[i] = ((time.time(),pickle.dumps(self.queue.get())))
                      self.count +=1
                self.n.write_blob2(self.table_name, tmp_list, auto_commit = False)
                self.n.commit_data()
            else:
                self.status = 3
                time.sleep(0.001)


    def print_locals(self):
        from std_msgs.msg import String
        pub = rospy.Publisher("dbstatus", String, queue_size = 10)
        while True:
            #print("================")
            #print("status : ", self.status)
            #print("hozon_now : ",self.hozon_now)
            #print("timestamp : ",self.timestamp)
            #print("newdb_name : ",self.newdb_name)
            #print("before_dbname : ",self.before_dbname)
            #print("qsize : ", self.queue.qsize())
            pub.publish("status : {} |hozon_now : {} |timestamp : {} |newdb_name : {} |before_dbname : {} |qsize : {} |num {} | obs {}".format(self.status, self.hozon_now, self.timestamp, self.newdb_name, self.before_dbname, self.queue.qsize(), self.scan_num, self.obs_mode))
            time.sleep(0.1)

    def commit(self):
        self.n2 = n2lite.xffts_logger(self.path_to_db)
        while True:
            self.n2.commit_data()
            time.sleep(5)
            print("commit")

if __name__ == "__main__":
    import numpy as np
    import pandas as pd
    import sys
    rospy.init_node(__file__[:-3], anonymous = True)
    #xffts_1 = xffts_logger("/media/amigos/HD-LCU3/test/test.db", "xffts", "XFFTS_SPEC")
    xffts_1 = xffts_logger("xffts")
    s1 = threading.Thread(target = xffts_1.save_to_db_tmp)
    s1.setDaemon(True)
    s1.start()
    debug_thread = threading.Thread(target = xffts_1.print_locals)
    debug_thread.setDaemon(True)
    debug_thread.start()
    xffts_1.main()
    rospy.spin()
