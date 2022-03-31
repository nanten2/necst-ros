#!usr/bin/env python3

# Shebang changed from ``python2`` (2022/3/16)
# Any reason older Python version was used?

import rospy
import time
import threading
import os
import glob
import sys

sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/amigos/Pictures/capture")
import cv2
import numpy
import camera
import rospkg

from PIL import Image

im = Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from necst.msg import oneshot_msg


class cam_controller(object):
    filename = ""

    def __init__(self):
        self.DLSR = camera.controller()
        self.DLSR.detect_camera()
        self.DLSR.set_whitebalance(white="SKY")
        self.DLSR.set_crop(crop="1.3x")
        print("ready")
        pass

    def start_thread(self):
        th = threading.Thread(target=self.pub_image)
        th.setDaemon(True)
        th.start()
        return

    def take_picture(self, req):
        print("start ROS_camera.py")
        # self.filename = str(req.time)+'.jpg'
        self.filename = req.filename + ".jpg"
        if os.path.exists("/home/amigos/Pictures/capture/" + self.filename) == True:
            pass
        else:
            self.DLSR.shutter_download(
                filename="/home/amigos/Pictures/capture/" + self.filename
            )
            print("oneshot!")
        return

    def pub_image(self):
        while True:
            if not self.filename == "":
                break
        while True:
            if os.path.exists("/home/amigos/Pictures/capture/" + self.filename) == True:
                break
            time.sleep(0.1)

        image_path = "/home/amigos/Pictures/capture/"
        img = cv2.imread(image_path + self.filename)
        bridge = CvBridge()
        pub = rospy.Publisher("Image", Image, queue_size=100, latch=True)
        pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
        print("publish picture")
        self.filename = ""
        self.remove_file()
        return

    def remove_file(self):  # no debug
        filelist = sorted(
            glob.glob("/home/amigos/Pictures/capture/*"),
            key=lambda f: os.stat(f).st_mtime,
        )
        if len(filelist) > 100:
            os.remove(filelist[0])
            print("delete file : ", filelist[0])
        else:
            pass
        return


if __name__ == "__main__":
    cam = cam_controller()
    rospy.init_node("camera_controller")
    cam.start_thread()
    sub = rospy.Subscriber("oneshot", oneshot_msg, cam.take_picture)
    rospy.spin()
