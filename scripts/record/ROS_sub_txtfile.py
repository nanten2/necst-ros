#!/usr/bin/env python3
import rospy
from necst.msg import textfile_msg

rospy.init_node("savetxt")


def callback(req):
    print(req)  # debug
    f = open(req.path, "w")
    f.write(req.data)
    f.close()


sub = rospy.Subscriber("text2", textfile_msg, callback, queue_size=10)
rospy.spin()
