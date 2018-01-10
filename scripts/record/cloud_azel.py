import os
import sys
import time
import threading

import rospy


### drive
sys.path.append("/home/necst/git")
from n2db import n2database
db = n2database.N2db()
db.authorize()

class record_cloud(object):

    azel_list = []

    def __init__(self):
        self.stop_thread = threading.Event()
        self.record_thread = threading.thread(target=self.list_count)
        self.record_thread.setDaemon(True)
        self.record_thread.start()
        return

    def add_list(self, req):
        _list = [req.time, req.az, req.el]
        self.azel_list.append(_list)
        return

    def list_count(self):
        while not rospy.is_shutdown():
            if len(self.azel_list) > 100: # 1count/0.1s
                self.record(self.azel_list)
                self.azel_list = []
            else:
                pass
        return

    def stop(self):
        self.record_thread.set()
        return

    def record(self):
        _list = self.azel_list
        db.authorize()
        for i in range(_list):
            db.INSERT(pjt='Telescope', table='AzEl', data=_list[i])
    


if __name__ == "__main__":
    rospy.init_node("cloud_AzEl")
    rec = record_cloud()
    rospy.Subscriber("save_cloud_azel", Save_cloud_azel, rec.add_list)
    rospy.spin()
