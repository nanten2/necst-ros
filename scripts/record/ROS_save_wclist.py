import rospy
import os
from necst.msg import List_coord_msg
from std_msgs.msg import String

rospy.init_node("save_wclist")

class save_wclist():
    home_path = "/home/amigos/hdd/data/"
    data_path = ""
    save_flag = False

    def __init__(self):
        sub1 = rospy.Subscriber("wc_list", List_coord_msg, self.callback, queue_size=10)
        sub2 = rospy.Subscriber("logger_path", String, self.callback2, queue_size=1)
        pass

    def callback(self, req):
        if self.save_flag:
            if not os.path.exists(self.data_path):
                os.makedirs(self.data_path)
            with open(os.path.join(self.data_path, "wc_list.txt"), "a") as f:
                f.write("{}#{}#{}#{}#{}#{}#{}#{}#{}\n".format(*req.x_list, *req.y_list, *req.time_list, req.coord,
                        req.off_az, req.off_el))
            print(req)
        else:
            print("not saving")
            pass

    def callback2(self, req):
        if req.data == "":
            self.save_flag = False
        else:
            self.data_path = os.path.join(self.home_path, req.data)
            self.save_flag = True


if __name__ == "__main__":
    s = save_wclist()
    rospy.spin()
