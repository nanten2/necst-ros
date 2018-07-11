from subprocess import Popen
import time
import threading
from datetime import datetime as dt
import sys
import rospy
from necst.msg import Bool_necst
sys.path.append("/home/amigos/git")
from n2db import n2database
db = n2database.N2db()
db.authorize()

current_num = 0
current_day = ""

import signal
def handler(signal, frame):
    obs_flag = False
    try:
        proc.send_signal(signal.SIGINT)
    except:
        pass
    print("**** sistem is down... ****")
    sys.exit()
signal.signal(signal.SIGINT, handler)
obs_flag = False

def _observation(req):
    global obs_flag
    obs_flag = req.data
    if obs_flag == False:
        try:
            proc.send_signal(signal.SIGINT)
        except:
            pass
    time.sleep(1.)
    return

def queue_check():
    global current_num
    global current_day
    ct = dt.utcnow()
    date = ct.strftime("%Y-%m-%d")
    try:
        data = db.SELECT(table="queue_list", start=date, end=date)
    except:
        #if no file
        return
    if current_day != date:
        current_num = 0
        current_day = date
    else:
        pass
    for i in range(current_num, len(data)):
        dh = int(data[i][0].split()[1].split(":")[0])
        dm = int(data[i][0].split()[1].split(":")[1])
        ds = int(data[i][0].split()[1].split(":")[2])
        dtime = dh*3600+dm*60+ds
        ct = dt.utcnow()
        hh = ct.hour
        mm = ct.minute
        ss = ct.second
        ctime = hh*3600+mm*60+ss
        if dtime>ctime:
            current_num = i+1
            while dtime>ctime and obs_flag==True:
                ct = dt.utcnow()
                hh = ct.hour
                mm = ct.minute
                ss = ct.second
                ctime = hh*3600+mm*60+ss                
                print("current time : ", ct.strftime("%H:%M:%S"))
                print("next observation is : ", str(data[i][0].split()[1]))
                print("\n")
                time.sleep(1.)
            if obs_flag == False:
                break
            return data[i][1]
        else:
            continue

    return 
        

rospy.init_node("queue_obs")
sub = rospy.Subscriber("queue_obs",Bool_necst, _observation, queue_size=1)

while True:
    print("start")
    if not obs_flag:
        print("no queue_list...\n")
    while not obs_flag:
        time.sleep(1.)
    filename = queue_check()
    
    if not filename:
        obs_flag = False
        print("no queue_list")
    else:
        print("start observation : ", filename)
        cmd = "python ROS_otf.py --obsfile " + str(filename)
        cmd = cmd.split()
        proc = Popen(cmd)
        proc.wait()
    print("end\n")


    

