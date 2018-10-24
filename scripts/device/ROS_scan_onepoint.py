#! /usr/bin/env python3

import rospy
from necst.msg import Move_mode_msg
from necst.msg import List_coord_msg
import time
from datetime import datetime as dt
import threading
import sys
sys.path.append("/home/amigos/ros/src/necst/lib/")
sys.path.append("/home/amigos/ros/src/necst/lib/")
import calc_offset

node_name = "worldcoordinate_onepoint"

class worldcoord(object):
    
    command = ""
    msg = ""
    
    def __init__(self):
        self.sub = rospy.Subscriber("onepoint_command", Move_mode_msg, self.note_command, queue_size=1)
        self.pub = rospy.Publisher("wc_list", List_coord_msg, queue_size=1)
        self.thread_start = threading.Thread(target=self.create_list)
        pass

    def note_command(self,req):
        if abs(req.x) > 360. or abs(req.y)>90:
            pass
        else:
            self.command = req
        return

    def create_list(self):
        msg = List_coord_msg()
        msg.from_node = node_name
        while not rospy.is_shutdown():
            command = self.command
            self.command = ""
            if command:
                print("start_create_list")

                ret = calc_offset.calc_offset([command.x], [command.y],
                                              command.coord,
                                              [command.off_x], [command.off_y],
                                              command.offcoord,
                                              command.dcos,
                                              [dt.fromtimestamp(command.timestamp)])
                if not ret:
                    continue
                current_time = time.time()

                msg.x_list = [ret[0], ret[0]]
                msg.y_list = [ret[1], ret[1]]
                msg.time_list = [command.timestamp, command.timestamp+3600.]
                msg.coord = command.coord
                msg.off_az = ret[2]
                msg.off_el = ret[3]
                msg.hosei = command.hosei
                msg.lamda = command.lamda
                msg.limit = command.limit
                msg.rotation = command.rotation
                msg.timestamp = current_time
                self.pub.publish(msg)
                print(msg)
                print("publish status!!\n")
                print("end_create_list\n")
            else:
                pass
            time.sleep(0.1)
        return

if __name__ == "__main__":
    rospy.init_node(node_name)
    wc = worldcoord()
    list_thread = threading.Thread(target=wc.create_list)
    list_thread.start()
    print("start calculation")
    
