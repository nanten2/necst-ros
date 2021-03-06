#!/usr/bin/env python3

import sys
import time
import rospy
import rosnode
from necst.msg import String_necst
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/necst/ros/src/necst/lib")
import topic_status

node_name = "authority_check"
deco = topic_status.deco(node_name)

class authority(object):

    authority = ""

    def __init__(self):
        self.pub = rospy.Publisher("authority_check", String_necst, queue_size = 1)
        self.sub = rospy.Subscriber("authority_regist", String_necst, self.registration, queue_size=1)
        return

    def registration(self, req):
        
        if req.data == "":
            if req.from_node == self.authority:
                self.authority = req.data
                print("release authority !!","\n")
            elif req.from_node == "master":
                self.authority = req.data
                print("obs_node is killed...")
            else:
                print("Can't release...")
                print("current authority :", self.authority,"\n")
                pass
        elif self.authority:
            print("current authority :", self.authority, "\n")
        else:
            self.authority = req.data
            print("change authority : ", self.authority,"\n")
            pass
        return

    @deco
    def authority_check(self):
        msg = String_necst()
        while not rospy.is_shutdown():
            self.node_alive()
            msg.data = self.authority
            self.pub.publish(msg)
            #print(msg)
            time.sleep(1.)
        return

    def node_alive(self):
        node_data = rosnode.get_node_names()
        if not self.authority:
            pass
        elif not ("/"+self.authority) in node_data:
            self.authority = ""
            print("old_node authority is removed..."+"\n")
        else:
            pass
        return
    

if __name__ == "__main__":
    rospy.init_node(node_name)
    print("start"+"\n")
    au = authority()
    au.authority_check()


