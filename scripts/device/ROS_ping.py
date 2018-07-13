#!/usr/bin/env python3

from subprocess import Popen, call, PIPE
from datetime import datetime as dt
import time
import sys
from necst.msg import Bool_necst
import rospy
node_name = "ping_check"
rospy.init_node(node_name)
pub = rospy.Publisher("status_ping", Bool_necst, queue_size=1)

import signal
def handler(signal, frame):
    print("****system is down...****")
    try:
        proc.send_signal(signal.SIGINT)
    except:
        pass
    pub.publish(False, node_name, time.time())        
    time.sleep(1.)
    sys.exit()
    pass
signal.signal(signal.SIGINT, handler)



print("start\n")
while True:
    #cmd = "ping 192.168.101.153" # test
    cmd = "ping jjy.nict.go.jp" # NICT
    cmd = cmd.split()
    proc = Popen(cmd,stdout=PIPE)
    aa = proc.stdout.readline()
    pub.publish(True, node_name, time.time())
    time.sleep(1.)
    proc.terminate()


