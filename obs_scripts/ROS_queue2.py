#!/usr/bin/env python3
import rospy
from necst.msg import Bool_necst
from necst.msg import String_necst
from necst.msg import String_necst

import time
import sys
from datetime import datetime as dt
from subprocess import Popen
import queue
obs_list = queue.PriorityQueue()

#sys.path.append("/home/amigos/git")
#from n2db import n2database
#db = n2database.N2db()
#db.authorize()
from firebase import firebase


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

queue_flag = False
stop_flag = False
additional_time = 0
proc = ""

# ----------
# callback
# ----------

def _web(req):
    global queue_flag
    global stop_flag
    if req.data == True:
        queue_flag = True
        #stop_flag = False
    else:
        queue_flag = False
        #stop_flag = True
        obs_stop()
    print("*******")
    return

def _stop(req):
    global stop_flag
    global proc
    stop_flag = req
    obs_stop()
    return

def obs_stop():
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
    rospy.Subscriber("queue_obs", Bool_necst, _web, queue_size=1)
    rospy.Subscriber("obs_stop", String_necst, _stop, queue_size=1)
    pub = rospy.Publisher("next_obs", String_necst,queue_size = 1)
    return

def read_conf():
    f = open("/home/amigos/tmp/pass.txt", "r")
    ff = f.readline()
    ff = ff.split()
    f.close()
    return ff[0],ff[1],ff[2]

def create_list():
    global obs_list
    now = dt.utcnow()
    name = now.strftime("queue_list/%Y/%m/%d")
    data = fb.get(name,None)
    if not data:
        return
    print(data)
    [obs_list.put(i) for i in data]
    return obs_list

def write_log(*data):
    now = dt.utcnow()
    date = now.strftime("%Y/%m/%d/%H:%M:%S")
    #db.authorize()
    #db.INSERT("Telescope", "Queue_log",[[date,target[4], target[5], "start"]])
    fb.put("","/queue_log/"+date,data)
    return

def select_target():
    global additional_time
    global obs_list
    print(obs_list.queue)
    utc = dt.utcnow()
    now = time.time()
    print(now)
    target = obs_list.get()
    #target = obs.split()    
    if float(target[3][0]) < now and float(target[3][1]) < now:
        return
    if now+300 < float(target[2][0]):# wait time 5[min]
        additional_time = float(target[2][0])-(now+300)
        priority = target[0]
        print("additional", additional_time)
    #elif now+300 < float(target[2][1]):
    #    additional_time = float(target[2][1])-(now+300)
    #    priority = target[0]
    #    print("additional", additional_time)
    else:
        priority = target[0]
        pass
    tmp_list = []
    change = 0
    print("ssssss",additional_time)
    print(priority)
    for i in range(len(obs_list.queue)):
        if additional_time < 180: # min operation time is 300[s] 300<500
            break
        tmp_target = obs_list.get()
        print("tmp",tmp_target)
        if float(tmp_target[6])*60*int(tmp_target[7]) < additional_time:
            tmp_obs = [priority-1]
            tmp_obs.extend(tmp_target[1:])
            tmp_list.append(tmp_obs)
            additional_time -= float(tmp_target[6])*60*int(tmp_target[7])
            change = 1
        elif float(tmp_target[6])*60 < additional_time:
            num = int(additional_time/(float(tmp_target[6])*60))
            tmp_obs1 = [priority-1]
            tmp_obs1.extend(tmp_target[1:-1])
            tmp_obs1.append(str(num))
            tmp_obs2 = tmp_target[:-1]
            tmp_obs2.append(str(int(tmp_target[7])-num))
            tmp_list.append(tmp_obs1)
            tmp_list.append(tmp_obs2)
            additional_time -= float(tmp_target[6])*60*int(num)
            change = 1
        else:
            tmp_list.append(tmp_target)
            change = 0
        print("add", additional_time)
        print(additional_time)
    if tmp_list:
        [obs_list.put(i) for i in tmp_list]
        #target = obs_list.get()
        #target = obs.split()
    else:
        pass
    if change:
        obs_list.put(target)        
        return
    else:
        pass

    print("observation")
    print("queue : ", obs_list.queue)
    if int(target[7]) > 1:
        #obs = obs.rsplit(" ",1)[0] +" "+ str(int(target[-1])-1)+"\n"
        tmp_obs = target[:-1]
        tmp_obs.append(int(target[-1])-1)
        print("tmp_obs", tmp_obs)
        print(type(tmp_obs))
        print(obs_list.queue)
        obs_list.put(tmp_obs)

    # wait observation
    additional_time = 0
    if now < float(target[3][0]):
        check = 0
    else:
        check = 1
    while now < float(target[2][check]):
        ut = dt.fromtimestamp(float(target[2][check]))
        print(target[5]+"current time : %s & next observation : %s" %(utc.strftime("%H:%M:%S"), ut.strftime("%H:%M:%S")))
        if stop_flag or not queue_flag:
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
        cmd+= " --obsfile "+ target[5] + " --plot_mode savefig"
    print("start observation : ", cmd)
    cmd = cmd.split()
    #try:
    write_log(target[4], target[5], "start")
    proc = Popen(cmd)
    proc.wait()
    write_log(target[4], target[5],"end")

    #except Exception as e:
        #print("parameter error")
        #rospy.logwarn(e)
    print("end observation")
    time.sleep(1)
    return

if __name__ == "__main__":
    initialize()
    url, passwd, mail = read_conf()
    fb = firebase.FirebaseApplication(url,None)
    auth = firebase.FirebaseAuthentication(passwd, mail, extra={"id":123})
    fb.authentication = auth

    data = create_list()
    if not data:
        print("no observation list")
        sys.exit()
    print("start queue observation program")
    while not rospy.is_shutdown():
        count = 0
        while not queue_flag or stop_flag:
        #while stop_flag:
            if count == 0:
                print("wait starting queue")
                count += 1
            time.sleep(1.)
        if not obs_list.queue:
            print("end queue observation")
            sys.exit()
        target = select_target()
        if not target:
            print("select target ...")
            continue
        observation(target)
        if stop_flag:
            print("stop observation")
            sys.exit()
        time.sleep(1.)
