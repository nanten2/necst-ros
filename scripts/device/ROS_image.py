#!/usr/bin/env python2

import time
import threading
import rospy
import numpy
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
sys.path.append('/home/amigos/Pictures/capture')
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as Imagemsg
#from necst.msg import shot_msg

class Image(object):
    
    filename = ''
    dirname = ''

    def __init__(self):
        pass
    
    def Image_save(self, req):
        print('subscribe picture')
        bridge = CvBridge()
        img_data = bridge.imgmsg_to_cv2(req, 'bgr8')
        #cv2.imshow(self.filename, img_data)
        cv2.imwrite(self.dirname + self.filename, img_data)
        print('save picture')
        self.filename = ''
        return

    def filename(self,req):
        self.filename = req.filename + '.jpg'
        self.dirname = req.dirname
        print(self.filename)
        return

if __name__ == '__main__':
    Image =image()
    rospy.init_node('Image_saver')
    sub1 = rospy.Subscriber('Image', Imagemsg, image.Image_save)
    sub2 = rospy.Subscriber('oneshot', oneshot_msg, Image.filename)
    print('waiting picture')
    rospy.spin()
