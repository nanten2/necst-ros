#!/usr/bin/env python
import csv
import pandas
import rospy
from necst.msg import Status_weather_msg
from necst.msg import xffts_flag_msg
import os

header_name = ["timestamp", "intemp", "outtemp", "inhumi", "outhumi", "winddir", "windsp",
               "press", "rain", "cabin_temp1", "cabin_temp2", "dome_temp1",
               "dome_temp2", "gen_temp1", "gen_temp2", "scan_num", "obs_mode"]

rospy.init_node("weather_logger")

class weather_logger():
    save_path = ""
    before_save_path = ""
    wd = 0
    scan_number = 0
    obs_mode = 0
    
    def initialize(self):
        with open(self.save_path, "w") as f:
            writer = csv.DictWriter(f, fieldnames=header_name)
            writer.writeheader()

    def write(self):
        print(self.save_path)
        with open(self.save_path, "a") as f:
            writer = csv.writer(f)
            writer.writerow([self.wd.timestamp, self.wd.in_temp, self.wd.out_temp, self.wd.in_humi, self.wd.out_humi, self.wd.wind_dir, self.wd.wind_sp,
                             self.wd.press, self.wd.rain, self.wd.cabin_temp1, self.wd.cabin_temp2, self.wd.dome_temp1, self.wd.dome_temp2,
                             self.wd.gen_temp1, self.wd.gen_temp2, self.scan_number, self.obs_mode])

    def update_weatherstatus(self, req):
        self.wd = req

    def callback(self, req):
        self.scan_number = req.scan_num
        self.obs_mode = req.obs_mode
        if req.newdb_name == "":
            return
        if not self.save_path == os.path.join(os.path.dirname(req.newdb_name), "weather.csv"):
            self.save_path = os.path.join(os.path.dirname(req.newdb_name), "weather.csv")
            self.initialize()
            self.write()
        else:
            self.write()

w = weather_logger()
sub = rospy.Subscriber("status_weather", Status_weather_msg, w.update_weatherstatus)
sub2 = rospy.Subscriber("XFFTS_DB_flag", xffts_flag_msg, w.callback, queue_size = 1)
rospy.spin()
