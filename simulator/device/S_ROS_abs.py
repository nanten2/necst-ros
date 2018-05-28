#!/usr/bin/env python3

import rospy
import time
import threading

from necst.msg import String_necst 
#import sys
#sys.path.append("/home/amigos/ros/src/necst/lib")
#import abs_device as dev

node_name = 'abs_controller'

class abs_controller(object):

    move_position = ""
    current_position = "IN"
    
    def __init__(self):
        self.sub1 = rospy.Subscriber('hot', String_necst, self._position, queue_size=1)
        self.sub2 = rospy.Subscriber('emergency', String_necst, self._emergency, queue_size=1)
        self.pub = rospy.Publisher('status_hot', String_necst, queue_size=1)
        pass

# -----------
# move
# -----------

    def _position(self, req):
        self.move_position = req.data.upper()
        return

    def _emergency(self,req):
        # no implementation
        rospy.loginfo('!!!emergency!!!')
        rospy.loginfo('!!!stop azel_move!!!')
        self.emergency_flag = 1
        return
    
    def move(self):
        while not rospy.is_shutdown():
            if self.move_position:
                #dev.move(self.move_position)
                self.current_position = self.move_position
                self.move_position = ""
            else:
                pass
            time.sleep(0.1)
        return
    
# ----------
# status
# ---------
    
    def status_thread(self):
        status_thread = threading.Thread(target=self.pub_status)
        status_thread.setDaemon(True)
        status_thread.start()
        return

    def pub_status(self):
        msg = String_necst()
        msg.from_node = node_name
        while not rospy.is_shutdown():
            position = self.current_position#dev.get_position()
            msg.timestamp = time.time()
            msg.data = position
            self.pub.publish(msg)
            time.sleep(0.1)
        return

if __name__ == '__main__':
    rospy.init_node(node_name)
    hot = abs_controller()
    hot.status_thread()
    hot.move()

