import time
import rospy
from necst.msg import Move_mode_msg
from std_msgs.msg import String

rospy.init_node("test")

class test1(object):
    #pub = rospy.Publisher("test",String,queue_size=1)
    #msg = String()
    #msg.data = "hello!!"
    pub = rospy.Publisher("test",Move_mode_msg,queue_size=1)
    msg = Move_mode_msg()

    def __init__(self):
        self.msg.x = 10
        self.msg.y = 20
        self.msg.coord = "j2000"
        self.msg.planet = 0
        self.msg.off_x = 10
        self.msg.off_y = 10
        self.msg.offcoord = "horizontal"
        self.msg.hosei="hosei_230.txt"
        self.msg.lamda=2600.
        self.msg.dcos=0
        self.msg.func_x=""
        self.msg.func_y=""
        self.msg.movetime=10
        self.msg.limit=True
        self.msg.assist=True
        self.msg.time = time.time()
        return

    def test1(self):
        for i in range(10000):        
            self.pub.publish(self.msg)
        print("finish")
        return        

def test2():
    pub = rospy.Publisher("test",String,queue_size=1)
    for i in range(10000):
        #pub = rospy.Publisher("test",Move_mode_msg,queue_size=1)
        #pub.publish(10,20,"j2000",0,10,10,"horizontal","hosei_230.txt",2600,0,"","",10,True,True,time.time())
        pub.publish("hello!!")
    print("finish")

if __name__=="__main__":
    test2()
