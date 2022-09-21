#!/usr/bin/env python3

import rospy
import time
import os 
import os
import sys
import numpy
sys.path.append('')
#from std_msgs.msg import String
#from std_msgs.msg import Float64
from necst.msg import Float64_necst

class listen(object):

    hensa = []

    def __init__(self):
        pass
        
    def callback(self, req):
        t2 = time.time()
        t1 = req.data
        t_hensa = t2 - t1
        if len(self.hensa) < 100:
            self.hensa.append(t_hensa)
            saveto = os.path.join('/home/amigos/data/', 'test.txt')
            f = open(saveto, 'a')
            f.write('%s \n'%(str(t_hensa)))
            print('!!!')
        else:
            sys.exit(-1)
        return

if __name__ == '__main__':
    listen = listen()
    rospy.init_node('subscliber')
    rospy.Subscriber('test', Float64_necst, listen.callback)
    rospy.spin()
    
