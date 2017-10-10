#!/usr/bin/env python
"""
2017/09/20 Shiotani
this script is sample script
main thread + 2 thread
"""
import rospy
import controller
import time
import threading

from ros_start.msg import NECST_msg
from std_msgs.msg import String

class nanten_main_controller(object):
    con = controller.controller()
    parameters = {
        'target_az':0,
        'target_el':0,
        'start_time':0,
        'flag':0
        }

    def __init__(self):
        th = threading.Thread(target = self.act_azel)
        th.setDaemon(True)
        th.start()
        th2 = threading.Thread(target = self.pub_status)
        th2.setDaemon(True)
        th2.start()

    def test(self):
        return

    def set_parameter(self,req):
        self.parameters[req.name] = req.value
        return

    def act_azel(self):
        for i in range(len(self.parameters['target_az'])):
            if self.emergency_flag:
                break
            else:
                pass
            self.con.azel_move(self.parameters['target_az'],self.parameters['target_el'],10000,12000)

    def emergency(self,req):
        rospy.loginfo('!!!emergency!!!')
        rospy.loginfo('!!!stop azel_move!!!')
        self.emergency_flag = 1
        return

    def pub_status(self):
        pub = rospy.Publisher('status',String, queue_size=10, latch = True)
        status = String()
        status.str = ####
        pub.publish(status.str)
        return

if __name__ == '__main__':
    n = nanten_main_controller()
    rospy.init_node('nanten_main_controller')
    n.test()
    sub = rospy.Subscriber('param_name', NECST_msg, n.set_parameter)
    sub = rospy.Subscriber('emergency', String, n.emergency)
    rospy.spin()
    
