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
from necst.msg import Status_weather_msg
sys.path.append("/home/amigos/ros/src/necst/lib/")
sys.path.append("/home/amigos/ros/src/necst/lib/")
import calc_coord
import calc_offset
import numpy#for debug

node_name = "worldcoordinate_otf_planet"

class worldcoord(object):

    command = ""
    msg = ""
    weather_data = 0

    def __init__(self):
        self.sub = rospy.Subscriber("horizontal_otf", Otf_mode_msg, self.note_command, queue_size=1)
        self.sub2 = rospy.Subscriber("status_weather", Status_weather_msg, self.update_weatherdata, queue_size=1)
        self.pub = rospy.Publisher("wc_list", List_coord_msg, queue_size=1)
        self.pub_obs_stop = rospy.Publisher("obs_stop", String_necst, queue_size=1)
        self.thread_start = threading.Thread(target=self.create_list)
        self.calc = calc_coord.azel_calc()
        pass

    def update_weatherdata(self, req):
        self.weather_data = req
        return

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
                start_x = command.off_x-float(command.dx)/2.-float(command.dx)/float(command.dt)*command.rampt
                start_y = command.off_y-float(command.dy)/2.-float(command.dy)/float(command.dt)*command.rampt
                total_t = command.rampt + command.dt * command.num
                end_x = command.off_x + command.dx * (command.num - 0.5)
                end_y = command.off_y + command.dy * (command.num - 0.5)
                time_list = [command.timestamp+command.delay, command.timestamp+command.delay+total_t]
                time_list = [dt.utcfromtimestamp(i) for i in time_list]

                ###中心座標Altazの計算
                press = self.weather_data.press
                temp = self.weather_data.out_temp
                humi = self.weather_data.out_humi/100
                lamda = 2600
                ret = self.calc.coordinate_calc([command.x*3600, command.x*3600], [command.y*3600, command.y*3600], time_list, "j2000", 0, 0, "hosei_230.txt", lamda, press, temp, humi)
                print("debug")
                print(numpy.array(ret)/3600)

                
                ### softlimit
                """
                if not all((0.<i<90. for i in altaz_list.alt.deg)):
                    self.pub_obs_stop.publish("This planet is not rizing...", node_name, time.time())
                    rospy.logerr("This planet is not rizing...")
                    continue
                else:
                    pass
                """
                msg.x_list = [ret[0][0] + start_x, ret[0][1] + end_x]
                msg.y_list = [ret[1][0] + start_x, ret[1][1] + end_y]
                print("x_list", msg.x_list)
                print("y_list", msg.y_list)
                current_time = time.time()
                msg.time_list = [command.timestamp+command.delay, command.timestamp+command.delay+total_t]
                #msg.coord = command.coord_sys
                msg.coord = "horizontal"
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
