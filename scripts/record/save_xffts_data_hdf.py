import h5py
import queue
import time
import rospy

class data_logger():
    def __init__(self):
        self.regist_subscriber()
        self.q = queue.Queue()
        self.c = 1
        pass

    def regist_subscriber(self):
        from nascorx_xffts.msg import XFFTS_msg
        from necst.msg import xffts_flag_msg
        self.sub1 = rospy.Subscriber("XFFTS_SPEC", XFFTS_msg, self.save_to_queue, queue_size = 100)
        #self.sub2 = rospy.Subscriber("XFFTS_DB_flag", xffts_flag_msg, self.flag, queue_size = 10)

    def save_to_queue(self, req):
        self.q.put(req)
        pass

    def save(self):
        tmplist = [0]*20
        self.f = h5py.File("./hdd/20190729_otfn31.hdf5", "a")
        #for i in range(20):
        #    self.f.create_group("array_set{}".format(i+1))
        self.f.create_group("timestamp")
        self.f.close()
        print("for debug")
        while not rospy.is_shutdown():
            self.f = h5py.File("./hdd/20190729_otfn31.hdf5", "a")
            st = time.time()
            if self.q.empty():
                continue
            d = self.q.get()
            for i in range(20):
                tmplist[i] = list(eval("d.SPEC_BE{}".format(i+1)))
            #for i in range(20):
            #    self.f.create_dataset("/array_set{}/spec{}".format(i+1, self.c), data = tmplist[i])
            self.f.create_dataset("spec{}".format(self.c), data = tmplist)
            self.f.create_dataset("/timestamp/time{}".format(self.c), data = float(d.timestamp))
            self.c+=1
            self.f.close()
            time.sleep(0.001)
            print("save : ", time.time()-st, self.q.qsize())

                
if __name__ == "__main__":
    rospy.init_node("test")
    d = data_logger()
    d.save()
    

    
