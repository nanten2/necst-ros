#!/usr/bin/env python3

import sys
import rospy
import time
import threading
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("home/necst/ros/src/necst/lib")
#import m4_device as dev

from necst.msg import String_necst

node_name = 'm4_controller'

class m4_controller(object):

    position = "IN"
    move_position = ''
    
    def __init__(self):
        self.sub = rospy.Subscriber('m4', String_necst, self._position, queue_size=1)
        self.sub = rospy.Subscriber('emergency', String_necst, self._emergency, queue_size=1)        
        self.pub = rospy.Publisher('status_m4',String_necst, queue_size=1)
        return


    # --------
    # move
    # --------

    def _position(self, req):
        if req.data.upper() in ( "IN", "NAGOYA"):
            self.move_position = "NAGOYA"
        elif req.data.upper() in  ("OUT", "SMART"):
            self.move_position = "SMART"
        else:
            rospy.logerr("!!command error!!")
            pass
        return
        
    def move(self):
        while not rospy.is_shutdown():
            if self.move_position:
                #dev.move(self.move_position)
                self.position = self.move_position
                self.move_position = ""
            else:
                pass
            time.sleep(0.1)
        return
            
    def stop(self):
        #dev.stop()
        return
        
    def _emergency(self,req):
        rospy.loginfo('!!!emergency!!!')
        rospy.loginfo('!!!stop M4!!!')
        self.stop()
        return            

    # --------
    # status
    # --------
    
    def status_thread(self):
        status = threading.Thread(target = self.pub_status)
        status.setDaemon(True)
        status.start()
        return

    def pub_status(self):
        msg = String_necst()
        msg.from_node = node_name
        while not rospy.is_shutdown():
            msg.data = self.position#dev.get_pos()
            msg.timestamp = time.time()
            self.pub.publish(msg)
            time.sleep(0.1)
        return

if __name__ == '__main__':
    rospy.init_node(node_name)
    m4 = m4_controller()    
    m4.status_thread()
    m4.move()
