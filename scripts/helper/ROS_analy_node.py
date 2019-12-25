#!/usr/bin/env python
import rospy
from necst.msg import analy_msg
from std_msgs.msg import String
import subprocess
import os
import shutil
import threading
import time

rospy.init_node("analy")

py_path = "/home/amigos/git/analy_n2data/note_py"
nb_path = "/home/amigos/git/analy_n2data/notebook"

analy_type = {"rsky": ["rsky.py", "rsky.ipynb"],
              "skydip": ["analy_skydip.py", "skydip.ipynb"],
              "edge": ["pointing_edge_xffts.py", "pointing_edge_xffts.ipynb"],
              "ps": ["Position_Switch.py", "Position_Switch.ipynb"],
              "simple_ps":["simple_ps.py", "simple_ps.ipynb"]}


status_analy = ""

def callback(req):
    global status_analy
    print(req)
    if not req.analy_type in analy_type:
        print("not find ...")#will update this comment
        return
    else:
        script_path = os.path.join(py_path, analy_type[req.analy_type][0])
    status_analy = req
    subprocess.run(['ipython', script_path, req.data_path])
    result_path = req.data_path.replace("data", "analysis")
    if not os.path.exists(result_path):
        os.makedirs(result_path, exist_ok=True)
    shutil.copy(os.path.join(nb_path, analy_type[req.analy_type][1]), result_path)
    status_analy = ""
    print("analy end")
    pass

def pub_status():
    pub = rospy.Publisher("analy_status", String, queue_size=10)
    s = String()
    while not rospy.is_shutdown():
        s.data = str(status_analy)
        pub.publish(s)
        time.sleep(0.5)

rospy.Subscriber("auto_analy", analy_msg, callback, queue_size=10)
th = threading.Thread(target=pub_status)
th.setDaemon(True)
th.start()
rospy.spin()
