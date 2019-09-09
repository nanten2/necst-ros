#!/usr/bin/env python
import rospy
from necst.msg import analy_msg
import subprocess
import os
import shutil

rospy.init_node("analy")

py_path = "/home/amigos/git/analy_n2data/note_py"
nb_path = "/home/amigos/git/analy_n2data/notebook"

analy_type = {"rsky": ["rsky.py", "rsky.ipynb"],
              "skydip": ["skydip.py", "skydip.ipynb"]}

def callback(req):
    print(req)
    if not req.analy_type in analy_type:
        print("not find ...")#will update this comment
        return
    else:
        script_path = os.path.join(py_path, analy_type[req.analy_type][0])
    subprocess.run(['ipython', script_path, req.data_path])
    shutil.copy(os.path.join(nb_path, analy_type[req.analy_type][1]), req.data_path)
    pass

rospy.Subscriber("auto_analy", analy_msg, callback, queue_size=10)
rospy.spin()
