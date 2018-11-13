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
    print("**** system is down... ****")
    sys.exit()
signal.signal(signal.SIGINT, handler)


# ----------
# parameter
# ----------

start_flag = False
stop_flag = False
additional_time = 0

# ----------
# callback
# ----------

def _start(req):
    global start_flag
    start_flag = req
    return

def _stop(req):
    global stop_flag
    stop_flag = req
    return

# ----------
# main
# ----------

def initialize():
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
    print("$$$$$$$$$$")
    print(obs_list.queue)
    utc = dt.utcnow()
    #now = float(utc.strftime("%H%M%S"))
    now = time.time()
    obs = obs_list.get()
    #print("ww")
    #print(obs[-1])# \n
    #print("ee")
    #print(obs[-2])# num
    #print("ss")
    #print(obs[-3])# space
    #print("dd")
    target = obs.split()    
    if float(target[3]) < now:
        #print("bad time")
        #print(float(target[1]) , now , float(target[2]))
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
        #print(obs_list)
        #print(float(tmp_target[6])*60)
        print("#####",tmp_target)
        if float(tmp_target[6])*60*int(tmp_target[7]) < additional_time:
            tmp_obs = "0x"+str(priority -1) + tmp_obs[3:]
            tmp_list.append(tmp_obs)
            additional_time -= float(tmp_target[6])*60*int(tmp_target[7])
            print("add1",additional_time)
        elif float(tmp_target[6])*60 < additional_time:
            num = int(additional_time/(float(tmp_target[6])*60))
            tmp_obs1 = "0x"+str(priority -1) + tmp_obs[3:-2]+str(num)+"\n"
            tmp_obs2 = tmp_obs[:-2]+str(int(tmp_target[7])-num)+"\n"
            tmp_list.append(tmp_obs1)
            tmp_list.append(tmp_obs2)
            additional_time -= float(tmp_target[6])*60*int(num)
            print("add2",additional_time)            
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
    #obs = obs_list.get()
    print(obs)
    #target = obs.split()
    print(target)
    print(target[7])
    if int(target[7]) > 1:
        obs = obs[:-2] + str(int(obs[-2])-1)+"\n"
        obs_list.put(obs)

    # wait observation
    additional_time = 0
    while now < float(target[2]):
        ut = dt.fromtimestamp(float(target[2]))
        print("current time : %s & next observation : %s" %(utc.strftime("%H:%M:%S"), ut.strftime("%H:%M:%S")))
        utc = dt.utcnow()
        #now = float(utc.strftime("%H%M%S"))
        now = time.time()
        time.sleep(1.)    
    return target

def observation(target):
    #print("target : ", target[3])
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
        rospy.logerr(e)
    print("end observation")
    time.sleep(1)
    return

if __name__ == "__main__":
    initialize()
    create_list()
    while not rospy.is_shutdown():
        if not obs_list.queue:
            print("end queue observation")
            sys.exit()
        target = select_target()
        if not target:
            print("no observation")
            continue
        observation(target)
        time.sleep(1.)
    
    
