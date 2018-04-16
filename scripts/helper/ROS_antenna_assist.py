#!/usr/bin/env python3

import time
import rospy
import threading
from std_msgs.msg import String
from necst.msg import Move_mode_msg
from necst.msg import Otf_mode_msg

class antenna_assist(object):
    x = 0
    y = 0
    coord = ""
    planet = 0
    off_x = 0
    off_y = 0
    offcoord = ''
    hosei = 'hosei_230.txt'
    lamda = 2600
    dcos = 0
    func_x = 0
    func_y = 0
    limit = True

    r_flag = 0
    g_flag = 0
    p_flag = 0
    old_time = 0
    switch = "nomal"

    def __init__(self):
        self.start_time = time.time()
        self.pub_nomal = rospy.Publisher("assist_antenna", Move_mode_msg, queue_size = 1, latch = True)
        self.pub_otf = rospy.Publisher("assist_otf", Otf_mode_msg, queue_size = 1, latch = True)
        rospy.Subscriber("move_stop", String, self.move_stop, queue_size=1)
        rospy.Subscriber("antenna_command", Move_mode_msg, self.antenna_assist, queue_size = 1)
        rospy.Subscriber('antenna_otf', Otf_mode_msg, self.otf_assist,queue_size = 1)
        pass

    def move_stop(self, req):
        self.start_time = time.time()
        return

    def start_thread(self):
        th1 = threading.Thread(target = self.pub_func)
        th1.setDaemon(True)
        th1.start()
        return

    def antenna_assist(self, req):
        """
        self.x = req.x
        self.y = req.y
        self.coord = req.coord
        self.planet = req.planet
        self.off_x = req.off_x
        self.off_y = req.off_y
        self.offcoord = req.offcoord
        self.hosei = req.hosei
        self.lamda = req.lamda
        self.dcos = req.dcos
        self.vel_x = req.vel_x
        self.vel_y = req.vel_y
        self.movetime = req.movetime
        self.limit = req.limit
        self.controller_time = req.time
        """
        self.switch = "nomal"
        self.param = req
        if req.time > self.start_time:
            self.r_flag = 1
        else:
            pass
        return

    def otf_assist(self, req):
         self.switch = "otf"
         self.param = req
         if req.time > self.start_time:
             self.r_flag = 1
         else:
             pass
         return

    def pub_func(self):
        while self.r_flag == 0 :
            time.sleep(0.1)
        else:
            while not rospy.is_shutdown():
                if self.param.assist == True:
                    self.pub_nomal.publish(self.param)
                    print('published')
                    print(self.param)
                elif self.param.assist == False:
                    now = self.param.time
                    if now != self.old_time:
                        if self.switch == "nomal":
                            self.pub_nomal.publish(self.param)
                        elif self.switch == "otf":
                            self.pub_otf.publish(self.param)
                        else:
                            pass
                    else:
                        pass
                    self.old_time = now
                    pass
                else:
                    pass
                #time.sleep(5)
                time.sleep(0.2)
                continue
        return
        
if __name__ == "__main__":
    rospy.init_node('antennna_assist')
    rospy.loginfo(" assist start ")
    at_as = antenna_assist()
    at_as.start_thread()
    rospy.spin()
    
