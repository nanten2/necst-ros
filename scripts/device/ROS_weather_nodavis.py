#!/usr/bin/env python3

import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import os
import datetime
import time
import pexpect
import getpass
import rospy
from necst.msg import Status_weather_msg
from ondo.msg import tr7nw_values
from davis.msg import davis_weather
from std_msgs.msg import Float64
from necst.msg import Float64_list_msg

node_name = "weather_status"

class weather_controller(object):
    #host = "amigos@200.91.8.66"
    host = "amigos@172.20.0.35"
    dir = "/home/weather/WeatherMonitor/Weather_Data/"
    #dir = "/home/necst/ros/src/necst/scripts/device/"
    copy_dir = "/home/amigos/data/monitor/"
    data = [0]*20
    passwd = ""

    humi24 = [0]*24
    wind24 = [0]*24
    
    press = 0
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

    cabin_hum = 0
    cabin_temp = 0
    dome_temp1 = 0
    dome_temp2 = 0    

    def __init__(self):
        #self.passwd = getpass.getpass()
        self.sub = rospy.Subscriber("outer_ondotori", tr7nw_values, self.get_ondotori)
        self.sub_davis = rospy.Subscriber("davis_weather", davis_weather, self.get_davis)
        self.sub_press = rospy.Subscriber("press_raspi", Float64, self.get_pressure)
        self.pub_humi = rospy.Publisher("web_humi24", Float64_list_msg, queue_size=1)
        self.pub_wind = rospy.Publisher("web_wind24", Float64_list_msg, queue_size=1)
        self.sub_hum1 = rospy.Subscriber("ondotori_hum", Float64, self.get_cabin_hum, queue_size=1)

        self.sub_ondo1 = rospy.Subscriber("ondotori_temp", Float64, self.get_cabin_temp, queue_size=1)
        self.sub_ondo1 = rospy.Subscriber("ondotori_dome_temp1", Float64, self.get_dome_temp1, queue_size=1)
        self.sub_ondo1 = rospy.Subscriber("ondotori_dome_temp2", Float64, self.get_dome_temp2, queue_size=1)                        
        pass

    def pub_func(self):
        pub = rospy.Publisher("status_weather", Status_weather_msg, queue_size = 10, latch = True)
        msg = Status_weather_msg()
        wmsg = Float64_list_msg()
        while not rospy.is_shutdown():
            #ret = self.get_weather()
            msg.in_temp = self.InTemp#ret[6]
            msg.out_temp = self.OutTemp#ret[7]
            #msg.out_temp = self.OutTemp_davis
            msg.in_humi = self.InHumi#ret[8]
            msg.out_humi = self.OutHumi#ret[9]
            #msg.out_humi = self.OutHumi_davis
            msg.wind_sp = self.WindSp#ret[11]
            msg.wind_dir = self.WindDir#ret[10]
            msg.press =self.press# ret[12]
            msg.rain = self.RainRate#ret[13]
            msg.cabin_temp1 = self.cabin_temp+273.15#ret[14] +273.15
            msg.cabin_temp2 = 296#ret[15] +273.15
            msg.dome_temp1 = self.dome_temp1+273.15#ret[16] +273.15
            msg.dome_temp2 = self.dome_temp2+273.15# ret[17] +273.15
            msg.gen_temp1 =0# ret[18] +273.15
            msg.gen_temp2 = 0#ret[19] +273.15
            msg.from_node = node_name
            msg.timestamp = time.time()
            pub.publish(msg)
            print(msg)

            now = time.gmtime()
            hour = now.tm_hour
            self.humi24[hour] = msg.out_humi
            self.wind24[hour] = msg.wind_sp
            wmsg.data = self.humi24
            self.pub_humi.publish(wmsg)
            wmsg.data = self.wind24
            self.pub_wind.publish(wmsg)
            
            time.sleep(1)
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
        self.OutTemp = req.ch1_value+273.15
        self.OutHumi = req.ch2_value
        return

    def get_pressure(self, req):
        self.press = req.data
        return

    def get_cabin_temp(self, req):
        self.cabin_temp = req.data
        return

    def get_cabin_hum(self, req):
        self.cabin_hum = req.data
        return

    def get_dome_temp1(self, req):
        self.dome_temp1 = req.data
        return

    def get_dome_temp2(self, req):
        self.dome_temp2 = req.data
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
