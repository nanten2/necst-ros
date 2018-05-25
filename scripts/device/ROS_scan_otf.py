#! /usr/bin/env python3

import rospy
from necst.msg import Otf_mode_msg
from necst.msg import List_coord_msg
import time
import threading
import sys
sys.path.append("/home/amigos/ros/src/necst/lib/")
sys.path.append("/home/amigos/ros/src/necst/lib/")
import calc_offset

node_name = "worldcoordinate_otf"

class worldcoord(object):

    command = ""
    msg = ""

    def __init__(self):
        self.sub = rospy.Subscriber("antenna_otf", Otf_mode_msg, self.note_command, queue_size=1)
        self.pub = rospy.Publisher("wc_list", List_coord_msg, queue_size=1)
        self.thread_start = threading.Thread(target=self.create_list)
        pass

    def note_command(self,req):
        print(req)
        self.command = req
        print(self.command)
        return

    def create_list(self):
        msg = List_coord_msg()
        msg.from_node = node_name
        while not rospy.is_shutdown():
            command = self.command
            self.command = ""
            if command:
                print("start_create_list")
                #ret = calc_offset.calc_offset(command.x, command.y, command.coord,
                #                              command.off_x, command.off_y, command.offcoord,
                #                              command.dcos)

                start_x = command.off_x-float(command.dx)/2.-float(command.dx)/float(command.dt)*command.rampt
                start_y = command.off_y-float(command.dy)/2.-float(command.dy)/float(command.dt)*command.rampt
                total_t = command.rampt + command.dt * command.num
                end_x = command.off_x + command.dx * (command.num - 0.5)
                end_y = command.off_y + command.dy * (command.num - 0.5)
                print(start_x, end_x, command.x)
                #off_dx_vel = (end_x - start_x) / total_t #(obs_end - obs_start)                                                                                    
                #off_dy_vel = (end_y - start_y) / total_t #(obs_end - obs_start)                                                                                    
                #x_list = [command.x+(start_x+off_dx_vel*i*0.1)/3600. for i in range(int(round(total_t/command.dt))*10)]
                #y_list = [command.y+(start_y+off_dy_vel*i*0.1)/3600. for i in range(int(round(total_t/command.dt))*10)]

                msg.x_list = [command.x*3600.+start_x, command.x*3600.+end_x]
                msg.y_list = [command.y*3600.+start_y, command.y*3600.+end_y]
                current_time = time.time()

                msg.time_list = [command.timestamp+command.delay, command.timestamp+command.delay+total_t]
                msg.coord = command.coord_sys
                msg.off_az = 0
                msg.off_el = 0
                msg.hosei = command.hosei
                msg.lamda = command.lamda
                msg.limit = command.limit
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
