#! /usr/bin/env python3

import rospy
from necst.msg import Move_mode_msg
from necst.msg import List_coord_msg
import time
import threading
import sys
sys.path.append("/home/amigos/ros/src/necst/lib/")
sys.path.append("/home/amigos/ros/src/necst/lib/")
import calc_offset

node_name = "worldcoordinate_linear"

class worldcoord(object):
    
    command = ""
    msg = ""
    
    def __init__(self):
        self.sub = rospy.Subscriber("linear_command", Move_mode_msg, self.note_command, queue_size=1)
        self.pub = rospy.Publisher("wc_list", List_coord_msg, queue_size=1)
        self.thread_start = threading.Thread(target=self.create_list)
        self.start_time = time.time()
        pass

    def note_command(self,req):
        if self.start_time > req.timestamp:
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

                for i in range(3601):
                    if -360.*3600. < command.x*3600.+command.off_x*i <360.*3600.:
                        num_x = i                        
                        pass
                    else:
                        break
                for i in range(3601):
                    if -90*3600. < command.y*3600.+command.off_y*i <90*3600.:
                        num_y = i                        
                        pass
                    else:
                        break
                delta_t = min(num_x, num_y)
                print("delta_t", delta_t)
                
                ret0 = calc_offset.calc_offset(command.x, command.y, command.coord,
                                              0, 0, command.offcoord, command.dcos)
                ret1 = calc_offset.calc_offset(command.x, command.y, command.coord,
                                              command.off_x*delta_t, command.off_y*delta_t,
                                              command.offcoord, command.dcos,)
                current_time = time.time()

                print(ret0,ret1)
                if command.coord  in ("horizontal","altaz"):
                    if command.coord == command.offcoord:
                        ret1[0]+=ret1[2]
                        ret1[1]+=ret1[3]
                    else:
                        pass
                else:
                    pass
                msg.x_list = [ret0[0], ret1[0]]
                msg.y_list = [ret0[1], ret1[1]]
                msg.time_list = [command.timestamp, command.timestamp+delta_t]
                msg.coord = command.coord
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
