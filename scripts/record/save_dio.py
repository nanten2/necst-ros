#!/usr/bin/env python3

from datetime import datetime as dt
import time
import os
import sys
import argparse
import rospy
from necst.msg import Float64_list_msg

node_name = "save_dio"

parser = argparse.ArgumentParser("save dio program")
parser.add_argument("name", help="input save filename")
param = parser.parse_args()
dirname = "/home/amigos/data/log/dio/"+param.name

dio = []

def _save(req):
    global dio
    dio = list(req.data)
    return

def check_file(day):
    try:
        if not os.path.exists(dirname):
            os.makedirs(dirname)
        else:
            pass
    except Exception as e:
        print(e)
        print("amigos can use this script!!")
        sys.exit()
    return

def save_file():
    while not rospy.is_shutdown():
        now = dt.utcnow()
        day = now.strftime("%Y%m%d")
        current = now.strftime("[UTC] %Y/%m/%d %H:%M:%S")

        if dio != []:
            check_file(day)        
            f = open(dirname+"/"+day+".txt", "a")
            f.write(current)
            f.write(" ")
            f.write(str(dio))
            f.write("\n")
            f.close()
            time.sleep(1.)
    return

if __name__ == "__main__":
    print("save dio\n")
    print("name : ", param.name)
    rospy.init_node(node_name)
    sub = rospy.Subscriber(param.name, Float64_list_msg, _save, queue_size = 1)
    save_file()
