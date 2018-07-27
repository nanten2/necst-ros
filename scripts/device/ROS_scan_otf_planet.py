#! /usr/bin/env python3

import rospy
from necst.msg import Otf_mode_msg
from necst.msg import List_coord_msg
from necst.msg import String_necst
import time
import threading
import sys
from astropy.coordinates import get_body, AltAz, EarthLocation, SkyCoord
from astropy.time import Time
import astropy.units as u
from datetime import datetime as dt
sys.path.append("/home/amigos/ros/src/necst/lib/")
sys.path.append("/home/amigos/ros/src/necst/lib/")
import calc_offset

node_name = "worldcoordinate_otf_planet"

class worldcoord(object):

    command = ""
    msg = ""
    planet_list = ['earth', 'sun', 'moon', 'mercury', 'venus', 'earth-moon-barycenter', 'mars', 'jupiter', 'saturn', 'uranus', 'neptune']

    def __init__(self):
        self.sub = rospy.Subscriber("planet_otf", Otf_mode_msg, self.note_command, queue_size=1)
        self.pub = rospy.Publisher("wc_list", List_coord_msg, queue_size=1)
        self.pub_obs_stop = rospy.Publisher("obs_stop", String_necst, queue_size=1)
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
        nanten2 = EarthLocation(lat = -22.96995611*u.deg, lon = -67.70308139*u.deg, height = 4863.85*u.m)
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

                if not command.planet.lower() in self.planet_list:
                    self.pub_obs_stop.publish("planet name is false...", node_name, time.time())
                    rospy.logerr("planet name is false...")
                    continue
                else:
                    pass
                
                time_list = [command.timestamp+command.delay, command.timestamp+command.delay+total_t]
                target_list = get_body(command.planet.lower(), Time(time_list))#gcrs
                target_list.location = nanten2
                altaz_list = target_list.altaz
                if not all((0.<i<90. for i in altaz_list.alt.deg)):
                    self.pub_obs_stop.publish("This planet is not rizing...", node_name, time.time())
                    rospy.logerr("This planet is not rizing...")
                    continue
                else:
                    pass
                ret_start = calc_offset.calc_offset(altaz_list.az.deg, altaz_list.alt.deg,
                                              "altaz",
                                              start_x, start_y, "altaz",
                                              command.dcos, time_list)
                ret_end = calc_offset.calc_offset(altaz_list.az.deg, altaz_list.alt.deg,
                                              "altaz",
                                              end_x, end_y, "altaz",
                                              command.dcos, time_list)                
                
                msg.x_list = [ret_start[0], ret_end[1]]
                msg.y_list = [ret_start[0], ret_end[1]]
                print("x_list", msg.x_list)
                print("y_list", msg.y_list)
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
