import rospy
from necst.msg import textfile_msg
from std_msgs.msg import String
import time
import os

path = "/home/amigos/ros/src/necst/lib/hosei_230.txt"
rospy.init_node("text_publisher")

def callback(req):
    print(req)
    f = open(req.data, "r")
    txt = f.read()
    f.close()
    s.data = txt
    s.path = "./{}".format(os.path.basename(req.data))
    print(txt)
    pub.publish(s)

#time.sleep(2)
pub = rospy.Publisher("text2", textfile_msg, queue_size=10)
sub = rospy.Subscriber("text1", String, callback, queue_size=1)
s = textfile_msg()

rospy.spin()
