#!/usr/bin/env python3
import time
import rospy
import rosnode
from necst.msg import String_necst
import atexit

class authority(object):

    auth = ""
    frame = "controller"
    node_name = ""

    def __init__(self):
        self.pub = rospy.Publisher("authority_regist", String_necst, queue_size=1)     
        self.sub = rospy.Subscriber("authority_check", String_necst, self.pick_up, queue_size=1)
        #atexit.register(self.finish)
        return

    def initialize(self):
        for i in range(100):
            name = self.frame +str(i)
            node_data = rosnode.get_node_names()
            if ("/" + name) in node_data:
                pass
            else:
                break
        self.node_name = name
        return name

    def start(self):
        self.registration(self.node_name)
        print("node_name is ", self.node_name)        
        return
            
    def pick_up(self,req):
        self.auth = req.data
        return
        
    def registration(self,name=""):
        msg = String_necst()
        msg.data = name
        msg.from_node = self.node_name
        msg.timestamp = time.time()
        self.pub.publish(msg)
        return
        
    def drop(self):
        self.registration("")
        return

    def deco_check(self, func):
        import functools
        @functools.wraps(func)
        def wrapper(*args,**kwargs):
            if self.auth == "":
                self.registration(self.node_name)
            else:
                pass
            time.sleep(0.5)
            if self.auth == self.node_name:        
                func(*args,**kwargs)
            else:
                print("This node don't have authority...")
                print("current authority : ", self.auth)                
                pass
        return wrapper


        
