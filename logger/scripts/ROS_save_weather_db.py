#!/usr/bin/env python3
import rospy
import datetime
import time
import os
import sys
import sqlite3
sys.path.append("~/git")
import n2lite
from necst.msg import Status_weather_msg

class log_weather():

    weather_status = ''
    
    def __init__(self):
        self.con = sqlite3.connect('/home/amigos/data/db/weather.db')
        self.cursor = self.con.cursor()
        self.cursor.execute("create table if not exists weather (timestamp int, in_temp float, out_temp float,in_humi float, out_humi float, wind_sp float, wind_dir, press float, rain float, cabin_temp1 float, cabin_temp2 float, dome_temp1 float, dome_temp2 float, gen_temp1 float, gen_temp2 float)")
        pass

    def callback(self, req):
        print('subscribe!')
        self.weather_status = req
        time.sleep(1)


    def logging(self):
        while self.weather_status == '':
            time.sleep(1)
        while not rospy.is_shutdown():
            now = datetime.datetime.now()
            ws = self.weather_status
            data = (ws.timestamp, ws.in_temp, ws.out_temp, ws.in_humi, ws.out_humi, ws.wind_sp, ws.wind_dir, ws.press, ws.rain, ws.cabin_temp1, ws.cabin_temp2, ws.dome_temp1, ws.dome_temp2, ws.gen_temp1, ws.gen_temp2)
            self.cursor.execute("insert into weather  values (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", data)
            print('ok!')
            self.con.commit()
            time.sleep(10)
    

if __name__ == '__main__':
    rospy.init_node('weather_log')
    lw = log_weather()
    rospy.Subscriber('status_weather', Status_weather_msg, lw.callback, queue_size = 1)
    lw.logging()
    rospy.spin()

