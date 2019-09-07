import rospy
from necst.msg import textfile_msg
import time
import os

rospy.init_node("text_publisher")

def callback(req):
    print(req)
    f = open(req.data, "r")
    txt = f.read()
    f.close()
    s.data = txt
    s.path = req.path
    print(txt)
    pub.publish(s)

pub = rospy.Publisher("text2", textfile_msg, queue_size=10)
sub = rospy.Subscriber("text1", textfile_msg, callback, queue_size=1)
s = textfile_msg()

rospy.spin()
