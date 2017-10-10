import threading
import time
import rospy
from std_msgs.msg import String
#from ros_start.msg import access_authority_msg
#from ros_start.msg import remote_authority_msg

class authority(object):
    authority = "release"

    def __init__(self):
        th = threading.Thread(target = self.authority_pub)
        th.setDaemon(True)
        th.start()
        
    def authority_pub(self):
        while True:
            pub = rospy.Publisher("authority_check", String, queue_size = 10)
            msg = String()
            msg.data = self.authority
            pub.publish(msg)
            rospy.loginfo(self.authority)
            time.sleep(0.01)

    def authority_change(self, req):
        self.authority = req.data
        print(authority)
        return


rospy.init_node("Authority")
au = authority()
print("start")
sub = rospy.Subscriber("authority_change", String, au.authority_change)
rospy.spin()

