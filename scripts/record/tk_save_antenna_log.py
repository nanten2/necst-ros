#!/usr/bin/env python3

import os
import time
from datetime import datetime as dt
import rospy
from necst.msg import List_coord_msg
from necst.msg import Status_encoder_msg
from necst.msg import Status_antenna_msg

enc_flag = False
enc_az = 0
enc_el = 0
com_flag = False
com_az = 0
com_el = 0

"""
def test(req):
    print("receive")
    xx = str(req.x_list[0])
    yy = str(req.y_list[0])
    tt = str(req.time_list[0])
    f = open("test2.txt","a")
    f.write(tt+" "+xx+" "+yy+"\n")
    f.close()
    return
"""

def enc(req):
    global enc_flag, enc_az, enc_el
    enc_az = req.enc_az
    enc_el = req.enc_el
    time.sleep(0.5)
    enc_flag = True    
    return
    
def com(req):
    global com_flag, com_az, com_el
    com_az = req.command_az
    com_el = req.command_el
    time.sleep(0.5)
    enc_flag = True    
    return

def save_status():
    now = dt.utcnow()
    dir_name = "/home/amigos/data/tk_log/"    
    if not os.path.exists(dir_name):
        print("create file")
        os.makedirs(dir_name)
    else:
        pass
    name = now.strftime("%Y%m%d_%H%M%S")
    if enc_flag and com_flag:
        print("wait receiving data...")
        time.sleep(1.)
    time.sleep(1.)
    f = open(dir_name + name+".txt", "a")
    print("start daving data")
    while not rospy.is_shutdown():
        now = time.time()
        f.write(str(now) +" "
                +str(com_az) +" "
                +str(com_el) +" "
                +str(enc_az) +" "
                +str(enc_el) +"\n")
        time.sleep(0.05)
    f.close()
    print("end")    
                

rospy.init_node("log_test")
print("start")
#sub = rospy.Subscriber("wc_list", List_coord_msg, test)
sub = rospy.Subscriber("status_encoder", Status_encoder_msg, enc, queue_size=1)
sub = rospy.Subscriber("status_antenna", Status_antenna_msg, com, queue_size=1)

save_status()
