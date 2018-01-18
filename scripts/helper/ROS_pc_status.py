#!/usr/bin/env python3
import psutil
import time
import rospy
import threading
import sys
from necst.msg import Status_pc_msg

rospy.init_node('cpu_memory_check', anonymous = True)
class pc_check(object):
    cpu_percent = 0
    used_memory = 0
    used_memory_percent = 0
    pc_name = ''

    def __init__(self):
        #print(sys.argv)
        try:
            self.pc_name = sys.argv[1]
        except:
            pass
        pass

    def thread_start(self):
        th = threading.Thread(target = self.get_status)
        th.setDaemon(True)
        th.start()

    def get_status(self):
        while True:
            mem = psutil.virtual_memory()
            #print('CPU percent : %5.2f %%  / Memory used : %d(%5.2f %%)'%(psutil.cpu_percent(),mem.used,mem.percent))
            self.cpu_percent = psutil.cpu_percent()
            self.used_memory = mem.used
            self.used_memory_percent = mem.percent
            time.sleep(10)
            continue

    def publish_data(self):
        pub = rospy.Publisher('pc_status', Status_pc_msg, queue_size = 1)
        while not rospy.is_shutdown():
            data = Status_pc_msg()
            data.time = time.ctime()
            data.cpu_per = self.cpu_percent
            data.pc_name = self.pc_name
            data.used_memory = int(self.used_memory)
            data.memory_per = self.used_memory_percent
            pub.publish(data)
            time.sleep(2)

if __name__ == '__main__':
    print('start')
    pc = pc_check()
    pc.thread_start()
    pc.publish_data()
    rospy.spin()
