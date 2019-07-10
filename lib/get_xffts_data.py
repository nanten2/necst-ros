import rospy
from nascorx_xffts.msg import XFFTS_msg
import numpy
import time

class get_xffts_data():
    def __init__(self):
        #rospy.init_node("getxfftsdata", anonymous = True)            
        self.spec = [None]*21
        self.count = 0
        self.regist_subscriber()
        pass

    def regist_subscriber(self):
        self.sub1 = rospy.Subscriber("XFFTS_SPEC", XFFTS_msg, self.callback, queue_size = 10)

    def callback(self, req):
        for i in range(20):
            self.spec[i] = eval("req.SPEC_BE{}".format(i+1))
        self.count += 1
        self.spec[20] = self.count
        pass

    def getdata(self):
        return self.spec

    def get_integdata(self, exposure):
        #paramter
        #exposure [sec]
        times = int(exposure/0.1)
        tmp = numpy.zeros((20,32768))
        b = 0
        count = 0
        while not rospy.is_shutdown():
            #print("W", b, self.spec[20])
            if not b == self.spec[20]:
                for i in range(20):
                    tmp[i] += self.spec[i]
                count+=1
                if count == times:
                    break
            else:
                pass
            b = self.spec[20]
            time.sleep(0.01)
        return tmp
            
        
        
        
