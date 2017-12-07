#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Bool

def stop(req):
    time.sleep(0.1)
    pub = rospy.Publisher("emergency", Bool, queue_size=10, latch=True)
    msg = Bool()
    msg.data = req.data
    pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("stop")
    sub = rospy.Subscriber("emergency_stop", Bool, stop)
    sub = rospy.Subscriber("interruption", Bool, stop)
    sub = rospy.Subscriber("limit", Bool, stop)
    rospy.spin()
