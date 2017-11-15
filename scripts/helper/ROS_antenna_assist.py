#!/usr/bin/env python

import time
import rospy
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
    

    def __init__(self):
        self.pub1 = rospy.Publisher("antenna_radec", Move_mode_msg, queue_size = 10, latch = True)
        self.pub2 = rospy.Publisher("antenna_galactic", Move_mode_msg, queue_size = 10, latch = True)
        self.pub3 = rospy.Publisher("antenna_planet", Move_mode_msg, queue_size = 10, latch = True)
        pass

    def radec_assist(self, req):
        self.ra = req.x
        self.dec = req.y
        self.code_mode = req.code_mode
        self.off_x = req.off_x
        self.off_y = req.off_y
        self.hosei = req.hosei
        self.offcoord = req.offcoord
        self.lamda = req.lamda
        self.dcos = req.dcos
        return

    def galactic_assist(self, req):
        self.l = req.x
        self.b = req.y
        self.off_x = req.off_x
        self.off_y = req.off_y
        self.hosei = req.hosei
        self.code_mode = req.code_mode
        self.offcoord = req.offcoord
        self.lamda = req.lamda
        self.dcos = req.dcos
        return

    def planet_assist(self, req):
        self.number = req.ntarg
        self.off_x = off_x
        self.off_y = off_y
        self.code_mode = req.code_mode
        self.hosei = req.hosei
        self.offcoord = req.offcoord
        self.lamda = req.lamda
        self.dcos = req.dcos
        return

    def pub_radec(self):
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
            rospy.loginfo(msg)
            self.pub1.publish(msg)
            time.sleep(5)
            continue

    def pub_galactic(self):
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
            rospy.loginfo(msg)
            self.pub2.publish(msg)
            time.sleep(5)
            continue

    def pub_planet(self):
        msg = Move_mode_msg()
        while not rospy.is_shutdown():
            msg.ntarg = self.number
            msg.off_x = self.off_x
            msg.off_y = self_off_y
            msg.code_mode = self.code_mode
            msg.hosei = self.hosei
            msg.offcoord = self.offcoord
            msg.lamda = self.lamda
            msg.dcos = self.dcos
            rospy.loginfo(msg)
            self.pub3.publish(msg)
            time.sleep(5)
            continue
        
if __name__ == "__main__":
    rospy.init_node('antennna_assist')
    rospy.loginfo(" assist start ")
    at_as = antenna_assist()
    rospy.Subscriber("antenna_radec", Move_mode_msg, at_as.radec_assist)
    rospy.Subscriber("antenna_galactic", Move_mode_msg, at_as.galactic_assist)
    rospy.Subscriber("antenna_planet", Move_mode_msg, at_as.planet_assist)
    at_as.pub_radec()
    at_as.pub_galactic()
    at_as.pub_planet()
    rospy.spin()
    
