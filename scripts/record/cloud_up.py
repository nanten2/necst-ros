import os
import sys
import time
import threading
import traceback
import rospy
from necst.msg import Status_encoder_msg

### drive
sys.path.append("/home/amigos/git")
from n2db import n2database
db = n2database.N2db()
db.authorize()

class record_cloud(object):

    #azel_list = []
    def __init__(self):
        pass
    """
    def start_upload(self):
        self.stop_thread = threading.Event()
        self.record_thread = threading.Thread(target=self.list_count)
        self.record_thread.setDaemon(True)
        self.record_thread.start()
        return
    """

    """
    def add_list(self, req):
        time.sleep(0.1)
        _list = [req.utc, req.enc_az/3600., req.enc_el/3600.]
        self.azel_list.append(_list)
        return
    """
    
    def list_count(self, data_list):
#        while not rospy.is_shutdown():
        if len(data_list) > 30: # 1count/0.1s
            #print(len(data_list), ' / 30')
            #azel_list = self.azel_list
            #self.azel_list = []
            return True
        else:
            #print(data_list)
          #  print(len(data_list), ' / 30')
           # time.sleep(0.0011)
            return False
    
    def stop(self):
        self.record_thread.set()
        return

    def record(self, azel_list):
        _list = azel_list
        db.authorize()
        a = time.time()
        print('start uploading...')
        try:
            db.INSERT(pjt='Telescope', table='AzEl', data=_list)
        except:
            traceback.print_exc()
            pass
        b = time.time()
        print('end  [%s : %5.2f sec]'%(len(_list), b-a))
        #print("time", b-a, "len", len(_list))

class upload_cloud(record_cloud):

    def __init__(self):
        pass
    
    def set_nodename(self, node_name):
        self.node_name = node_name
        pass
    
    def set_pjt_table(self, pjt, table):
        self.pjt = pjt
        self.table = table
        print('set project name')
        pass
        
if __name__ == "__main__":
    rospy.init_node("cloud_AzEl")
    rec = record_cloud()
    print("Subscribe start")
    rospy.Subscriber("status_encoder", Status_encoder_msg, rec.add_list, queue_size = 1)
    rospy.spin()
