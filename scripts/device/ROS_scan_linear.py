#! /usr/bin/env python3

import rospy
from necst.msg import Move_mode_msg
from necst.msg import List_coord_msg
from necst.msg import Bool_necst
import time
import threading
from datetime import datetime as dt
import astropy.units as u
from astropy.coordinates import SkyCoord, EarthLocation
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/amigos/ros/src/necst/lib")
import calc_offset
import topic_status

node_name = "worldcoordinate_linear"
deco = topic_status.deco(node_name)


latitude = -22.96995611
longitude = -67.70308139
height = 4863.85
nanten2 = EarthLocation(lat = latitude*u.deg, lon = longitude*u.deg, height = height*u.m)

class worldcoord(object):
    
    command = ""
    msg = ""
    move_on = 0
    
    def __init__(self):
        self.sub = rospy.Subscriber("linear_command", Move_mode_msg, self.note_command, queue_size=1)
        self.pub = rospy.Publisher("wc_list", List_coord_msg, queue_size=1)
        self.thread_start = threading.Thread(target=self.create_list)
        self.start_time = time.time()
        self.pub_stop = rospy.Publisher("move_stop", Bool_necst, queue_size = 1, latch = True)
        pass

    def note_command(self,req):
        if self.start_time > req.timestamp:
            pass
        else:
            self.command = req
        return
    @deco
    def create_list(self):
        msg = List_coord_msg()
        msg.from_node = node_name
        while not rospy.is_shutdown():
            command = self.command
            self.command = ""
            if command:

                num_x = 0
                num_y = 0
                try:
                    coord = command.coord.replace("horizontal", "altaz")
                except:
                    coord = command.coord
                try:
                    offcoord = command.offcoord.replace("horizontal", "altaz")
                except:
                    offcoord = command.offcoord
                coord = SkyCoord(command.x, command.y, frame=coord, unit="deg", location=nanten2, obstime=dt.utcnow())

                _coord = coord.transform_to(offcoord)
                for i in range(3601):
                    if -360.*3600. <= command.off_x*i <= 360.*3600.:
                        num_x = i
                        pass
                    else:
                        break
                    if -90*3600. < _coord.data.lat.arcsec+command.off_y*i < 90*3600.:
                        num_y = i                        
                        pass
                    else:
                        break
                delta_t = min(num_x, num_y)
                print(delta_t)
                off_x_list = [command.off_x*delta_t/4*i for i in range(5)]
                off_y_list = [command.off_y*delta_t/4*i for i in range(5)]
                time_list = [dt.fromtimestamp(command.timestamp+delta_t/4*i) for i in range(5)]
                ret = calc_offset.calc_offset(
                    [command.x for i in range(5)],
                    [command.y for i in range(5)],
                    command.coord,
                    off_x_list,
                    off_y_list,
                    command.offcoord, command.dcos,
                    time_list)
                if not ret:
                    continue
                current_time = time.time()

                print(ret[0],ret[1])

                msg.x_list = ret[0]
                msg.y_list = ret[1]
                msg.time_list = [command.timestamp+delta_t/4*i for i in range(5)]
                msg.coord = command.coord
                msg.off_az = 0
                msg.off_el = 0
                msg.hosei = command.hosei
                msg.lamda = command.lamda
                msg.limit = command.limit
                msg.timestamp = current_time
                self.pub.publish(msg)
                self.move_on = command.timestamp+delta_t+0.1
                print(msg)
                print("publish status!!\n")
                print("end_create_list\n")
            else:
                if 0 < self.move_on < time.time() :
                    self.pub_stop.publish(True, node_name, time.time())
                    print("emergency stop!!")
                    self.move_on = 0
                    pass
                else:
                    pass
                pass
            time.sleep(0.1)
        return

if __name__ == "__main__":
    rospy.init_node(node_name)
    wc = worldcoord()
    list_thread = threading.Thread(target=wc.create_list)
    list_thread.start()
    print("start calculation")
