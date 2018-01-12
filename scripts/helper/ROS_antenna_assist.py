#!/usr/bin/env python3

import time
import rospy
import threading
from necst.msg import Move_mode_msg
from necst.msg import Otf_mode_msg

class antenna_assist(object):
    ra = 0
    dec = 0

    l = 0
    b = 0

    number = 0
    
    code_mode = ''
    off_x = 0
    off_y = 0
    offcoord = ''
    hosei = 'hosei_230.txt'
    lamda = 2600
    dcos = 0
    limit = True

    r_flag = 0
    g_flag = 0
    p_flag = 0
    

    def __init__(self):
        self.start_time = time.time()
        self.pub1 = rospy.Publisher("antenna_radec", Move_mode_msg, queue_size = 10, latch = True)
        self.pub2 = rospy.Publisher("antenna_galactic", Move_mode_msg, queue_size = 10, latch = True)
        self.pub3 = rospy.Publisher("antenna_planet", Move_mode_msg, queue_size = 10, latch = True)
        pass

    def start_thread(self):
        th1 = threading.Thread(target = self.pub_radec)
        th1.setDaemon(True)
        th1.start()
        th2 = threading.Thread(target = self.pub_galactic)
        th2.setDaemon(True)
        th2.start()
        th3 = threading.Thread(target = self.pub_planet)
        th3.setDaemon(True)
        th3.start()
        return

    def radec_assist(self, req):
        self.r_flag = 1
        self.ra = req.x
        self.dec = req.y
        self.code_mode = req.code_mode
        self.off_x = req.off_x
        self.off_y = req.off_y
        self.hosei = req.hosei
        self.offcoord = req.offcoord
        self.lamda = req.lamda
        self.dcos = req.dcos
        self.limit = req.limit
        self.controller_time =req.time
        return

    def galactic_assist(self, req):
        self.g_flag = 1
        self.l = req.x
        self.b = req.y
        self.off_x = req.off_x
        self.off_y = req.off_y
        self.hosei = req.hosei
        self.code_mode = req.code_mode
        self.offcoord = req.offcoord
        self.lamda = req.lamda
        self.dcos = req.dcos
        self.limit = req.limit
        self.controller_time =req.time
        return

    def planet_assist(self, req):
        self.p_flag = 1
        self.number = req.ntarg
        self.off_x = req.off_x
        self.off_y = req.off_y
        self.code_mode = req.code_mode
        self.hosei = req.hosei
        self.offcoord = req.offcoord
        self.lamda = req.lamda
        self.dcos = req.dcos
        self.limit = req.limit
        self.controller_time =req.time
        return

    def pub_radec(self):
        while self.r_flag == 0 :
            time.sleep(0.1)
        else:
            msg = Move_mode_msg()
            while not rospy.is_shutdown():
                msg.x = self.ra
                msg.y = self.dec
                msg.code_mode = self.code_mode
                msg.off_x = self.off_x
                msg.off_y = self.off_y
                msg.hosei = self.hosei
                msg.offcoord = self.offcoord
                msg.lamda = self.lamda
                msg.dcos = self.dcos
                msg.limit = self.limit
                rospy.loginfo(msg)
                if self.start_time > self.controller_time :
                    continue
                else:
                    pass
                self.pub1.publish(msg)
                time.sleep(5)
                continue
        return

    def pub_galactic(self):
        while self.g_flag == 0:
            time.sleep(1)
        else:
            msg = Move_mode_msg()
            while not rospy.is_shutdown():
                msg.x = self.l
                msg.y = self.b
                msg.off_x = self.off_x
                msg.off_y = self.off_y
                msg.code_mode = self.code_mode
                msg.offcoord = self.offcoord
                msg.lamda = self.lamda
                msg.hosei = self.hosei
                msg.dcos = self.dcos
                msg.limit = self.limit
                rospy.loginfo(msg)
                if self.start_time > self.controller_time :
                    continue
                else:
                    pass
                self.pub2.publish(msg)
                time.sleep(5)
                continue
        return

    def pub_planet(self):
        while self.p_flag == 0:
            time.sleep(1)
        else:
            msg = Move_mode_msg()
            while not rospy.is_shutdown():
                msg.ntarg = self.number
                msg.off_x = self.off_x
                msg.off_y = self.off_y
                msg.code_mode = self.code_mode
                msg.hosei = self.hosei
                msg.offcoord = self.offcoord
                msg.lamda = self.lamda
                msg.dcos = self.dcos
                msg.limit = self.limit
                rospy.loginfo(msg)
                if self.start_time > self.controller_time :
                    continue
                else:
                    pass
                self.pub3.publish(msg)
                time.sleep(5)
                continue
        return
        
if __name__ == "__main__":
    rospy.init_node('antennna_assist')
    rospy.loginfo(" assist start ")
    at_as = antenna_assist()
    at_as.start_thread()
    rospy.Subscriber("antenna_radec", Move_mode_msg, at_as.radec_assist, queue_size = 1)
    rospy.Subscriber("antenna_galactic", Move_mode_msg, at_as.galactic_assist)
    rospy.Subscriber("antenna_planet", Move_mode_msg, at_as.planet_assist)
    rospy.spin()
    
