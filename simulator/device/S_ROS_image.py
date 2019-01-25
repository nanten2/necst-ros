#!/usr/bin/env python2

import time
import threading
import rospy
import numpy
import os
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
sys.path.append('/home/amigos/Pictures/capture')
import cv2
from cv_bridge import CvBridge
from PIL import Image as i
from sensor_msgs.msg import Image as Imagemsg
from necst.msg import oneshot_msg
import subprocess

class Image(object):
    filename = ''
    dirname = ''
    shot_mode = ''
    
    def __init__(self):
        pass
    
    def Image_save(self, req):
        if os.path.exists(self.dirname + self.filename) == True:
            return
        else:
            print('subscribe picture')
            bridge = CvBridge()
            img_data = bridge.imgmsg_to_cv2(req, 'bgr8')
            if not os.path.exists(self.dirname):
                os.makedirs(self.dirname)
            cv2.imwrite(self.dirname + self.filename, img_data)            
            print(self.dirname, self.filename)
            print('save picture')
            if self.shot_mode == 'oneshot':
                img = i.open(self.dirname + self.filename)
                img.show()
            elif self.shot_mode == 'all_sky':
                pass
            self.filename = ''
            return

    def dif_file(self,req):
        self.filename = req.filename + '.jpg'
        self.dirname = req.dirname

        if os.path.exists(self.dirname + self.filename) == True:
            return
        else:
            if not os.path.exists(self.dirname):
                os.makedirs(self.dirname)
            else:
                pass
        print(self.dirname)
        ret = subprocess.call(["cp", "/home/amigos/data/experiment/all_sky_shot/moon.jpg",self.dirname+self.filename])
        print(ret)
        
        print("end shot")
        return ret

if __name__ == '__main__':
    print("ccd simulator start")
    image =Image()
    rospy.init_node('Image_saver')
    #sub1 = rospy.Subscriber('Image', Imagemsg, image.Image_save)
    sub2 = rospy.Subscriber('oneshot', oneshot_msg, image.dif_file)
    print('waiting picture')
    rospy.spin()
