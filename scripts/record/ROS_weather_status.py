#!/usr/bin/env python3
import rospy
import datetime
import time
import os
import sys
from necst.msg import Status_weather_msg

class log_weather():

    weather_status = ''
    
    def __init__(self):
        pass

    def callback(self, req):
        self.weather_status = req
        time.sleep(1)


    def logging(self):
        while self.weather_status == '':
            time.sleep(1)
            
        while not rospy.is_shutdown():
            now = datetime.datetime.now()
            save_dir = '/home/amigos/log/{}/{}/{}'.format(now.year, now.month, now.day)

            if not os.path.exists(save_dir):
                os.makedirs(save_dir, exist_ok=True)
            f = open('{}/weather_status.txt'.format(save_dir), 'a')
            ws = self.weather_status
            t_stamp = datetime.datetime.utcfromtimestamp(ws.timestamp)
            log = '\r{} {} {} {} {} {} {} {} {} {} {} {} {} {} {}'.format(ws.timestamp, ws.in_temp, ws.out_temp, ws.in_humi, ws.out_humi, ws.wind_sp, ws.wind_dir, ws.press, ws.rain, ws.cabin_temp1, ws.cabin_temp2, ws.dome_temp1, ws.dome_temp2, ws.gen_temp1, ws.gen_temp2)
            f.write(log)
            f.close()
            #sys.stdout.write('\r{:.1f} {:2.1f} {:2.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f}'.format(ws.in_temp, ws.out_temp, ws.in_humi, ws.out_humi, ws.wind_sp, ws.wind_dir, ws.press, ws.rain, ws.cabin_temp1, ws.cabin_temp2, ws.dome_temp1, ws.dome_temp2, ws.gen_temp1, ws.gen_temp2))
            #sys.stdout.flush()
            
            time.sleep(10)
    

if __name__ == '__main__':
    rospy.init_node('weather_log')
    lw = log_weather()
    rospy.Subscriber('status_weather', Status_weather_msg, lw.callback, queue_size = 1)
    lw.logging()
    rospy.spin()

