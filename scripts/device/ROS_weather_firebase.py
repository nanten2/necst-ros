#!/usr/bin/env python3

import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import os
import datetime
import time
import pexpect
import getpass
import threading
import rospy
from necst.msg import Status_weather_msg
from ondo.msg import tr7nw_values
from davis.msg import davis_weather
from firebase import firebase
fb = firebase.FirebaseApplication("https://nasco-weather.firebaseio.com",None)
auth = firebase.FirebaseAuthentication("2wxODlRK5Mh4Ci8BGLN67hiVnTVzv4jo5eC0xYS4", "nascofirebase@gmail.com", extra={"id":123})
fb.authentication = auth

node_name = "weather_status"

class weather_controller(object):
    host = "amigos@200.91.8.66"
    dir = "/home/weather/WeatherMonitor/Weather_Data/"
    #dir = "/home/necst/ros/src/necst/scripts/device/"
    copy_dir = "/home/amigos/data/monitor/"
    data = [0]*20
    passwd = ""
    
    # from ondotori
    OutTemp = 0
    OutHumi = 0
    
    # from davis_VantagePro
    Press = 0
    InTemp = 0
    InHumi = 0
    OutTemp_davis = 0
    OutHumi_davis = 0
    WindSp = 0
    WindDir = 0
    RainRate = 0

    def __init__(self):
        #self.passwd = getpass.getpass()
        self.sub = rospy.Subscriber("outer_ondotori", tr7nw_values, self.get_ondotori)
        self.sub_davis = rospy.Subscriber("davis_weather", davis_weather, self.get_davis)
        pass

    def pub_func(self):
        pub = rospy.Publisher("status_weather", Status_weather_msg, queue_size = 10, latch = True)
        msg = Status_weather_msg()
        self.firebase_thread()
        while not rospy.is_shutdown():
            #ret = self.get_weather()
            msg.in_temp = self.InTemp#ret[6]
            #msg.out_temp = self.OutTemp#ret[7]
            msg.out_temp = self.OutTemp_davis
            msg.in_humi = self.InHumi#ret[8]
            #msg.out_humi = self.OutHumi#ret[9]
            msg.out_humi = self.OutHumi_davis
            msg.wind_sp = self.WindSp#ret[11]
            msg.wind_dir = self.WindDir#ret[10]
            msg.press = self.Press#ret[12]
            msg.rain = self.RainRate#ret[13]
            #msg.cabin_temp1 = ret[14]
            #msg.cabin_temp2 = ret[15]
            #msg.dome_temp1 = ret[16]
            #msg.dome_temp2 = ret[17]
            #msg.gen_temp1 = ret[18]
            #msg.gen_temp2 = ret[19]
            msg.from_node = node_name
            msg.timestamp = time.time()
            pub.publish(msg)
            print(msg)
            time.sleep(1)
        return

    def firebase_thread(self):
        th = threading.Thread(target=self.pub_firebase)
        th.start()
        return
    
    def pub_firebase(self):
        while not rospy.is_shutdown():
            print("test")
            fb.put("", "/NECST/Monitor/Instrument/Weather",{"InTemp":self.InTemp, "OutTemp":self.OutTemp_davis, "InHumi":self.InHumi,"OutHumi":self.OutHumi_davis, "WindSp":self.WindSp, "WindDir":self.WindDir, "Press":self.Press, "Rain":self.RainRate})
            time.sleep(1.)
        return

    def copy_file(self, _dir, data):
        path = self.copy_dir + _dir
        if not os.path.exists(path):
            os.makedirs(path)
            print("make_dir")
        access = pexpect.spawn('scp %s:%s %s' % (self.host, self.dir+data, self.copy_dir+data))
        access.expect('.*ssword:')
        access.sendline(self.passwd)
        access.interact()
        time.sleep(0.1)
        return

    def get_ondotori(self,req):
        self.OutTemp = req.ch1_value
        self.OutHumi = req.ch2_value
        return

    def get_davis(self, req):
        if req.error_check == "Normal":
            self.Press = req.press
            self.InTemp = req.in_temp
            self.InHumi = req.in_humi
            self.OutTemp_davis = req.out_temp
            self.OutHumi_davis = req.out_humi
            self.WindSp = req.wind_sp
            self.WindDir = req.wind_dir
            self.RainRate = req.rain_rate
        else:# if req.error_check == "Error"
            print("Can not access weather station")
            pass
        return

    def get_weather(self):
        now = time.time()#-17*3600.#-17h(nanmeteo time)
        d = datetime.datetime.utcfromtimestamp(now)
        
        if d.month < 10:
            month = '0'+str(d.month)
        else:
            month = str(d.month)
            
        if d.day < 10:
            day = '0'+str(d.day)
        else:
            day = str(d.day)

        data = str(d.year)+month+"/"+str(d.year)+month+day+".nwd"
        self.copy_file(str(d.year)+month + "/", data)

        with open(self.copy_dir+data, "r") as f:
            last_data = f.readlines()[-1]

        data_list = last_data.strip()
        data_list = data_list.split(",")
        for i in range(len(data_list)):
            self.data[i] = float(data_list[i].strip())

        return self.data

if __name__ == "__main__":
    rospy.init_node(node_name)
    wc = weather_controller()
    wc.pub_func()