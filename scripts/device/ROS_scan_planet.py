#! /usr/bin/env python3

import rospy
from necst.msg import Move_mode_msg
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

node_name = "worldcoordinate_planet"

class worldcoord(object):
    
    command = ""
    msg = ""
    planet_list = ['earth', 'sun', 'moon', 'mercury', 'venus', 'earth-moon-barycenter', 'mars', 'jupiter', 'saturn', 'uranus', 'neptune']
    
    def __init__(self):
        self.sub = rospy.Subscriber("planet_command", Move_mode_msg, self.note_command, queue_size=1)
        self.pub = rospy.Publisher("wc_list", List_coord_msg, queue_size=1)
        self.pub_obs_stop = rospy.Publisher("obs_stop", String_necst, queue_size=1)
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
        nanten2 = EarthLocation(lat = -22.96995611*u.deg, lon = -67.70308139*u.deg, height = 4863.85*u.m)
        list_num = 600
        delta_t = 1 #[s]
        while not rospy.is_shutdown():
            command = self.command
            self.command = ""
            if command:
                print("start_create_list")
                pass
            else:
                time.sleep(0.1)
                continue
            #print(command.planet)
            time_list = [dt.fromtimestamp(command.timestamp+delta_t*i) for i in range(list_num)]
            #time_list = Time(time_list)

            if not command.planet.lower() in self.planet_list:
                self.pub_obs_stop.publish("planet name is false...", node_name, time.time())
                rospy.logerr("planet name is false...")
                continue
            else:
                pass
            target_list = get_body(command.planet.lower(), Time(time_list))#gcrs
            #print(target_list)
            target_list.location = nanten2
            altaz_list = target_list.altaz
            if not all((0.<i<90. for i in altaz_list.alt.deg)):
                self.pub_obs_stop.publish("This planet is not rizing...", node_name, time.time())                
                rospy.logerr("This planet is not rizing...")
                continue
            ret = calc_offset.calc_offset(altaz_list.az.deg, altaz_list.alt.deg,
                                          "altaz",
                                          command.off_x, command.off_y, command.offcoord,
                                          command.dcos, time_list)
            
            current_time = time.time()

            msg.x_list = ret[0]
            msg.y_list = ret[1]
            msg.time_list = [command.timestamp+delta_t*i for i in range(list_num)]
            msg.coord = "planet"
            msg.off_az = ret[2]
            msg.off_el = ret[3]
            msg.hosei = command.hosei
            msg.lamda = command.lamda
            msg.limit = command.limit
            msg.timestamp = current_time
            self.pub.publish(msg)
            print(msg.time_list)
            print("publish status!!\n")
            print("end_create_list\n")
        return

if __name__ == "__main__":
    rospy.init_node(node_name)
    wc = worldcoord()
    list_thread = threading.Thread(target=wc.create_list)
    list_thread.start()
    print("start calculation")
    

