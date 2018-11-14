#!/usr/bin/env python3
import rospy
from necst.msg import Bool_necst
from necst.msg import String_necst
from necst.msg import String_necst

import time
import sys
from datetime import datetime as dt
from subprocess import Popen
import ast
import queue
obs_list = queue.PriorityQueue()

# ----------
# handler
# ----------

import signal
def handler(signal, frame):
    global obs_flag
    obs_flag = False
    try:
        proc.send_signal(signal.SIGINT)
    except:
        pass
    print("**** queue system is down... ****")
    sys.exit()
signal.signal(signal.SIGINT, handler)


# ----------
# parameter
# ----------

start_flag = False
stop_flag = False
additional_time = 0
proc = ""

# ----------
# callback
# ----------

def _start(req):
    global start_flag
    global stop_flag
    start_flag = req
    stop_flag = False
    print("*******")
    return

def _stop(req):
    global stop_flag
    global proc
    stop_flag = req
    try:
        proc.send_signal(signal.SIGINT)
        proc.terminate()
    except:
        pass

    return

# ----------
# main
# ----------

def initialize():
    rospy.init_node("queue_observation")
    rospy.Subscriber("queue_obs", Bool_necst, _start, queue_size=1)
    rospy.Subscriber("obs_stop", String_necst, _stop, queue_size=1)
    pub = rospy.Publisher("next_obs", String_necst,queue_size = 1)
    return

def create_list():
    global obs_list
    f = open("queue.txt","r") ###tmp
    f.readline() # trash
    ff = f.readlines()
    #print(ff)
    now = time.time()
    [obs_list.put(i) for i in ff if float(i.split()[3])>now]
    return obs_list

def select_target():
    global additional_time
    global obs_list
    print(obs_list.queue)
    utc = dt.utcnow()
    now = time.time()
    obs = obs_list.get()
    target = obs.split()    
    if float(target[3]) < now:
        return
    if now+300 < float(target[2]):# wait time 5[min]
        additional_time = float(target[2])-(now+300)
        priority = ast.literal_eval(target[0])
        print("additional", additional_time)
    tmp_list = []
    for i in range(len(obs_list.queue)):
        if additional_time < 180: # min operation time is 300[s] 300<500
            break
        tmp_obs = obs_list.get()
        tmp_target = tmp_obs.split()
        if float(tmp_target[6])*60*int(tmp_target[7]) < additional_time:
            tmp_obs = "0x"+str(priority -1) + tmp_obs[3:]
            tmp_list.append(tmp_obs)
            additional_time -= float(tmp_target[6])*60*int(tmp_target[7])
        elif float(tmp_target[6])*60 < additional_time:
            num = int(additional_time/(float(tmp_target[6])*60))
            ltmp_obs = tmp_obs.rsplit(" ",1)[0]
            tmp_obs1 = "0x"+str(priority -1) +" "+ ltmp_obs.split(" ",1)[1] + " " +str(num)+"\n"
            tmp_obs2 = ltmp_obs+" "+str(int(tmp_target[7])-num)+"\n"
            tmp_list.append(tmp_obs1)
            tmp_list.append(tmp_obs2)
            additional_time -= float(tmp_target[6])*60*int(num)
        else:
            tmp_list.append(tmp_obs)
    if tmp_list:
        [obs_list.put(i) for i in tmp_list]
        obs_list.put(obs)
        obs = obs_list.get()
        target = obs.split()
    else:
        pass

    print("observation")
    print(obs_list.queue)
    if int(target[7]) > 1:
        obs = obs.rsplit(" ",1)[0] +" "+ str(int(target[-1])-1)+"\n"
        obs_list.put(obs)

    # wait observation
    additional_time = 0
    while now < float(target[2]):
        ut = dt.fromtimestamp(float(target[2]))
        print("current time : %s & next observation : %s" %(utc.strftime("%H:%M:%S"), ut.strftime("%H:%M:%S")))
        if stop_flag:
            print("stop observation!!")
            return
        utc = dt.utcnow()
        now = time.time()
        time.sleep(1.)    
    return target

def observation(target):
    global proc
    cmd = "python "+target[4]
    if target[5]:
        cmd+= " --obsfile "+ target[5]
    print("start observation : ", cmd)
    cmd = cmd.split()
    try:
        proc = Popen(cmd)
        proc.wait()
    except Exception as e:
        print("parameter error")
        rospy.logwarn(e)
    print("end observation")
    time.sleep(1)
    return

if __name__ == "__main__":
    initialize()
    create_list()
    print("start queue observation")
    while not rospy.is_shutdown():
        #while not start_flag or stop_flag:
        while stop_flag:            
            time.sleep(1.)
        if not obs_list.queue:
            print("end queue observation")
            sys.exit()
        target = select_target()
        if not target:
            print("no observation")
            continue
        observation(target)
        if stop_flag:
            print("stop observation")
            sys.exit()
        time.sleep(1.)
    
    
