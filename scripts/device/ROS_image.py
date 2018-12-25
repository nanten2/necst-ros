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
from sensor_msgs.msg import Image as Imagemsg
from necst.msg import oneshot_msg

class Image(object):
    filename = ''
    dirname = ''

    def __init__(self):
        pass
    
    def Image_save(self, req):
        if os.path.exists(self.dirname + self.filename) == True:
            return
        else:
            print('subscribe picture')
            bridge = CvBridge()
            img_data = bridge.imgmsg_to_cv2(req, 'bgr8')
            cv2.imwrite(self.dirname + self.filename, img_data)
            print('save picture')
            img = cv2.imread(self.dirname + self.filename)
            cv2.imshow(self.dirname + self.filename, img)
            cv2.waitKey(0)
            cv2.destroyAllwindows()
            
            self.filename = ''
            return

    def dif_file(self,req):
        self.filename = req.filename + '.jpg'
        self.dirname = req.dirname
        print(self.dirname + self.filename)
        return

if __name__ == '__main__':
    image =Image()
    rospy.init_node('Image_saver')
    sub1 = rospy.Subscriber('Image', Imagemsg, image.Image_save)
    sub2 = rospy.Subscriber('oneshot', oneshot_msg, image.dif_file)
    print('waiting picture')
    rospy.spin()
