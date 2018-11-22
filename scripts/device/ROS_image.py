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
    

    def __init__(self):
        pass
    
    def Image_view(self, req):
        print('subscribe picture')
        ut = time.gmtime()
        self.filename = str(ut) + '.jpg'
        bridge = CvBridge()
        img_data = bridge.imgmsg_to_cv2(req, 'bgr8')
        img = cv2.imread(self.filename, 1)
        cv2.imshow(self.filename, img_data)
        print('push [s] key to preserve image')
        if cv2.waitkey(1) == ord('s'):
            cv2.imwrite('/home/amigos/Pictures/capture/'+ self.filename, img)
            cv2.destroyAllWindows()
        self.filename = ''
        return

if __name__ == '__main__':
    Image =Image()
    rospy.init_node('Image')
    #Image.start_thread()
    sub = rospy.Subscriber('Image', Imagemsg, Image.Image_view)
    print('waiting picture')
    #sub2 = rospy.Subscriber('shot', shot_msg, Image.dec_filename)
    rospy.spin()
