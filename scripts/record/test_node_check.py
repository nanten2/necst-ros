import rospy
import rosnode
import time
import threading
from necst.msg import Status_node_msg
from datetime import datetime as dt
from std_msgs.msg import String

"""
------------------------------------------------
[History]
2018/02/05 : kondo 
------------------------------------------------
"""

class node(object):

    node_list = {'/antenna_server':[],
                 '/controller_client':[],
                 '/antenna_move':[],
                 '/pc_status':[],
                 '/tracking':[],
                 '/m4_controller':[],
                 '/check_alert':[],
                 '/weather_status':[],
                 '/Status':[],
                 '/test':[],
                 '/dome_tracking':[],
                 '/rosout':[],
                 '/antenna_assist':[],
                 '/alert':[],
                 '/m2_controller':[],
                 '/abs_controller':[],
                 '/dome':[],
                 '/encoder_status':[]}

    dic = {}

    display = True
    
    def __init__(self):
        pass
    
    def thread_start(self):
        node_check_thread = threading.Thread(target=self.node_check)
        node_check_thread.start()
        display_thread = threading.Thread(target=self.display)
        display_thread.start()
        return

    def display(self):
        while not rospy.is_shutdown():
            menu = input()
            if menu.lower() == "d":
                self.display = False
                time.sleep(1.)
                print("\n\n\n")
                name = input("topic_name : ")
                print(name, self.dic[name])
                #self.sub = rospy.Subscriber(name, String, self.callback)

                while not rospy.is_shutdown():
                    close = input("display close ?(y/n) : ")
                    if close=="y":
                        self.display = True
                        break
                    else:
                        pass
            else:
                pass
            
    def callback(self, req):
        print(req)
        self.sub.unregister()
        return
            

    def node_check(self):
        while not rospy.is_shutdown():
            current_time = dt.utcnow().strftime("%Y/%m/%d %H:%M:%S")
            current_list = rosnode.get_node_names()
            node_list = self.node_list
            diflist = set(node_list) - set(current_list)
            diflist = list(diflist)

            status = ""
            status += "current time  " + str(current_time) + "\n"
            status += "running node_name"+"   "+ "publisher_name"+"\n\n"
            for i in current_list:
                space = " "*(20-len(i))
                status += str(i)+space+str(node_list[i])+"\n"
            if self.display:
                print(status)
                print("\n")
            else:
                pass
            if diflist:
                rospy.logwarn("This node is not running !!\n")
                rospy.logwarn(diflist)
                print("\n")

            time.sleep(1.)
        return

    def callback(self, req):
        if not (req.pub_name in self.node_list["/"+req.node_name]):
            self.node_list["/"+req.node_name].append(req.pub_name)
            self.dic[req.pub_name] = req.frame_name

if __name__ == "__main__":
    rospy.init_node("test")
    node = node()
    sub = rospy.Subscriber("status_node", Status_node_msg, node.callback)
    node.thread_start()

    
