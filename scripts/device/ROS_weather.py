#!/usr/bin/env python
import datetime
import time
import rospy
from necst.msg import Status_weather_msg

class weather_controller(object):
    host = "weather@200.91.8.66"
    #dir = "/home/weather/WeatherMonitor/Weather_Data/"
    dir = "/home/amigos/ros/src/necst/ROS/"
    data = [0]*20

    def __init__(self):
        rospy.init_node("weather_status")
        pass

    def pub_func(self):
        pub = rospy.Publisher("status_weather", Status_weather_msg, queue_size = 10, latch = True)
        msg = Status_weather_msg()
        while not rospy.is_shutdown():
            ret = self.get_weather()
            msg.in_temp = ret[6]
            msg.out_temp = ret[7]
            msg.in_humi = ret[8]
            msg.out_humi = ret[9]
            msg.wind_sp = ret[10]
            msg.wind_dir = ret[11]
            msg.press = ret[12]
            msg.rain = ret[13]
            msg.cabin_temp1 = ret[14]
            msg.cabin_temp2 = ret[15]
            msg.dome_temp1 = ret[16]
            msg.dome_temp2 = ret[17]
            msg.gen_temp1 = ret[18]
            msg.gen_temp2 = ret[19]
            pub.publish(msg)
            print(msg)
            time.sleep(1)
        return


    def get_weather(self):
        now = time.time()
        d = datetime.datetime.utcfromtimestamp(now)
        
        if d.month < 10:
            month = '0'+str(d.month)
        else:
            month = str(d.month)
            
        if d.day < 10:
            day = '0'+str(d.day)
        else:
            day = str(d.day)

        '''
        data = str(d.year)+month+"/"+str(d.year)+month+day+".nwd"
        ret = subprocess.check_output(["ssh", self.host, "tail", self.dir+data, "-n", "1"])
        res = ret.split()
        for i in range(20):
            self.data[i] = res[i].decode("UTF-8").strip(',')
            '''
        #data = str(d.year)+month+"/"+str(d.year)+month+day+".nwd"
        data = str(2016)+str(12)+str(19)+".nwd" 
        f = open(self.dir+data, "r")
        last_data = f.readlines()[-1]
        f.close()
        print(f)
        data_list = last_data.strip()
        for i in range(20):
            self.data[i] = float(data_list.split()[i].strip(","))

        return self.data

if __name__ == "__main__":
    wc = weather_controller()
    wc.pub_func()
