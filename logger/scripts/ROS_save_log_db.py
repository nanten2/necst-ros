#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from necst.msg import Status_obs_msg
from datetime import datetime
import sys

sys.path.append("/home/amigos/git")
import n2lite
import time

path_to_db = "/home/amigos/data/db/ctrl_log.db"
path_to_db2 = "/home/amigos/data/db/observation_log.db"
a = n2lite.N2lite(path_to_db)
b = n2lite.N2lite(path_to_db2)
before_req = 0


def save(callback):
    now = datetime.utcnow()
    data = callback.data.split("#")
    a.write("log2", "", [time.time(), data[0], data[1], data[2]])
    a.commit_data()
    print("write")


def save_obs_status(req):
    global before_req
    print("subscrlibe!")
    now = datetime.now()
    time_ = now.strftime("%Y/%m/%d %H:%M:%S")
    if before_req == 0:
        pass
    # elif not (before_req.active == req.active and before_req.obs_script == req.obs_script):
    #    pass
    # else:
    #    return

    if req.active == True:
        b.write(
            "observation_log",
            "",
            [time_, req.target, req.obs_script, req.obs_file, "START"],
        )
    else:
        b.write(
            "observation_log",
            "",
            [time_, req.target, req.obs_script, req.obs_file, "END"],
        )
    before_req = req
    b.commit_data()
    print(time_, " commit data")
    """
    ###Observation start
    if req.active == True and before_active == False:
        b.write("observation_log", "", [time_, req.target, req.obs_script, req.obs_file, "START"])
        print('pub!')
        before_active = True
        ###Initialize or Finalize
        if req.target.split(' ')[0] in ['initialize', 'finalize']:
            pass
    ###Observation end
    elif req.active == False and before_active == True:
        b.write("observation_log", "", [time_, req.target, req.obs_script, req.obs_file, "END"])
        print('pub!2')
        before_active = False
    """


if __name__ == "__main__":
    a.make_table("log2", {"time": "float", "log1": "str", "log2": "str", "log3": "str"})
    b.make_table(
        "observation_log",
        {
            "time": "str",
            "target": "str",
            "obs_script": "str",
            "obs_file": "str",
            "status": "str",
        },
    )
    rospy.init_node("save_log_db")
    sub1 = rospy.Subscriber("logging_ctrl", String, save, queue_size=10)
    sub2 = rospy.Subscriber(
        "obs_status", Status_obs_msg, save_obs_status, queue_size=10
    )
    rospy.spin()
