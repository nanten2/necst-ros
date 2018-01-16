#!/usr/bin/env python3

import time
import rospy
import threading
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
    vel_x = 0
    vel_y = 0
    limit = True

    r_flag = 0
    g_flag = 0
    p_flag = 0
    

    def __init__(self):
        self.start_time = time.time()
        self.pub = rospy.Publisher("assist_antenna", Move_mode_msg, queue_size = 1, latch = True)
        pass

    def start_thread(self):
        th1 = threading.Thread(target = self.pub_antenna)
        th1.setDaemon(True)
        th1.start()
        return

    def antenna_assist(self, req):
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
        self.controller_time =req.time
        if self.controller_time > self.start_time:
            self.r_flag = 1
        else:
            pass
        return

    def pub_antenna(self):
        while self.r_flag == 0 :
            time.sleep(0.1)
        else:
            while not rospy.is_shutdown():
                self.pub.publish(self.x, self.y, self.coord, self.planet, self.off_x, self.off_y, self.offcoord, self.hosei, self.lamda, self.dcos, self.vel_x, self.vel_y, self.movetime, self.limit, self.controller_time)
                print('published')
                print(self.x, self.y, self.coord, self.planet, self.off_x, self.off_y, self.offcoord, self.hosei, self.lamda, self.dcos, self.vel_x, self.vel_y, self.limit, self.controller_time)
                time.sleep(5)
                continue
        return
        
if __name__ == "__main__":
    rospy.init_node('antennna_assist')
    rospy.loginfo(" assist start ")
    at_as = antenna_assist()
    at_as.start_thread()
    rospy.Subscriber("antenna_command", Move_mode_msg, at_as.antenna_assist, queue_size = 1)
    rospy.spin()
    
