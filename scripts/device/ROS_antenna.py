#!/usr/bin/env python3

import sys
import rospy
import time
import math
import threading
from std_msgs.msg import String
from necst.msg import Velocity_mode_msg
from necst.msg import Move_mode_msg
from necst.msg import Otf_mode_msg
from necst.msg import Status_encoder_msg
from necst.msg import Status_weather_msg
from necst.msg import list_azelmsg
from datetime import datetime,timedelta
sys.path.append("/home/necst/ros/src/necst/lib")
sys.path.append("/home/amigos/ros/src/necst/lib")
import azel_calc

class antenna(object):
    
    off_list = {"off_az":0, "off_el":0, "off_j2000_ra":0, "off_j2000_dec":0, "off_b1950_ra":0, "off_b1950_dec":0,  "off_l":0, "off_b":0}
    longitude = -67.70308139
    #longitude = -67.70308139*math.pi/180
    latitude = -22.96995611
    #latitude = -22.96995611*math.pi/180
    height = 4863.85
    dut1 = -0.20489  #delta UT:  UT1-UTC (UTC seconds)

    enc_az = ""
    enc_el = ""
    temp = 300#""
    press = 500#""
    humi = 0.80#""

    
    limit = 0
    stime = 0
    calc_delay = 2.

    soft_limit_az = 240.
    soft_limit_up = 80.
    soft_limit_down = 20.

    az_list = ""
    el_list = ""
    limit = True
    start_time = ""
    
    def __init__(self):
        self.calc = azel_calc.azel_calc()
        self.stime = time.time()
        self.pub = rospy.Publisher("list_azel", list_azelmsg, queue_size = 1,)
        rospy.Subscriber('status_weather', Status_weather_msg, self.note_weather)
        rospy.Subscriber("move_stop", String, self.move_stop)
        time.sleep(3.)
        rospy.Subscriber("status_encoder", Status_encoder_msg, self.note_encoder)
        rospy.Subscriber('assist_antenna', Move_mode_msg, self.antenna_move)
        rospy.Subscriber('assist_otf', Otf_mode_msg, self.otf_start)

    def note_encoder(self, req):
        self.enc_az = req.enc_az
        self.enc_el = req.enc_el
        return

    def note_weather(self, req):
        self.temp = req.out_temp #[deg_C]
        self.press = req.press/100. #[Pa]
        self.humi = req.out_humi/100. #[%]
        return
        
    def move_stop(self, req):
        self.stime = time.time()
        rospy.logwarn("move stop!!")
        return
        
    def server_start(self):
        rospy.loginfo(" Read ok ")
        azel_thread = threading.Thread(target=self.azel_publish)
        azel_thread.start()
        return
        
    def azel_publish(self):
        while not rospy.is_shutdown():
            if self.start_time:
                if self.limit:
                    ret = self.limit_check(self.az_list, self.el_list)
                else:
                    ret = ""
                    pass
                if ret:
                    rospy.logerr("Publish False...")
                else:
                    self.pub.publish(az_list=self.az_list, el_list=self.el_list, start_time=self.start_time)
                    rospy.loginfo('Publish ok.')
                    print("\n")
                    pass

                self.az_list = self.el_list = self.start_time = ""
                self.limit = True
            else:
                pass
            time.sleep(1)
        return

    def antenna_move(self, req):
        if req.time < self.stime:
            pass
        elif not self.temp:
            rospy.logerr("weather_node is not move!!")
        else:
            print("start calculation")
            now = datetime.utcnow() + timedelta(seconds=self.calc_delay)
            if req.coord.lower() == "horizontal":
                ret = self.calc.azel_calc(req.x, req.y,
                                          req.off_x, req.off_y,
                                          req.offcoord, now, req.func_x, req.func_y, req.movetime)
            else:
                ret = self.calc.coordinate_calc(req.x, req.y, req.coord, req.planet,
                                                req.off_x, req.off_y,
                                                req.offcoord, req.hosei, req.lamda,
                                                req.dcos, self.temp, self.press, self.humi,
                                                now, req.movetime)

                pass
            
            self.limit = req.limit
            self.az_list = ret[0]
            self.el_list = ret[1]
            self.start_time = ret[2]

            print("end calculation")
        return

    def otf_start(self, req):
        if not self.temp:
            rospy.logerr("weather_node is not move!!")
        else:
            print("start calculation")
            start_x = req.off_x-float(req.dx)/2.-float(req.dx)/float(req.dt)*req.rampt
            start_y = req.off_y-float(req.dy)/2.-float(req.dy)/float(req.dt)*req.rampt
            total_t = req.rampt + req.dt * req.num
            end_x = req.off_x + req.dx * (req.num - 0.5)
            end_y = req.off_y + req.dy * (req.num - 0.5)
            obs_start =  datetime(req.start_on[0], req.start_on[1], req.start_on[2], req.start_on[3], req.start_on[4], req.start_on[5], req.start_on[6]) - timedelta(seconds=float(req.rampt))
            #obs_end = obs_start + timedelta(seconds=float(total_t))
            off_dx_vel = (end_x - start_x) / total_t #(obs_end - obs_start)
            off_dy_vel = (end_y - start_y) / total_t #(obs_end - obs_start)

            x_list = [req.x+(start_x+off_dx_vel*i*0.1)/3600. for i in range(int(round(total_t/req.dt))*10)]
            y_list = [req.y+(start_y+off_dy_vel*i*0.1)/3600. for i in range(int(round(total_t/req.dt))*10)]
            
            ret = self.calc.coordinate_calc(x_list, y_list, req.coord_sys,
                                            req.num, req.off_x, req.off_y,
                                            req.offcoord, req.hosei, req.lamda,
                                            req.dcos, self.temp, self.press,
                                            self.humi, obs_start, req.movetime)
            self.limit = req.limit
            self.az_list = ret[0]
            self.el_list = ret[1]
            self.start_time = ret[2]
            #print("###################")
            #print(list(map(lambda x : x*3600., x_list)))
            #print("\n\n\n\n")
            #print(list(map(lambda x : x*3600., y_list)))
            print("end calculation")

        return 

    def limit_check(self, az_list, el_list):
        for i in az_list:
            if i > self.soft_limit_az*3600.:
                limit_az = "!!cw 1st soft limit!!"
            elif i < -self.soft_limit_az*3600.:
                limit_az = "!!ccw 1st soft limit!!"
            else:
                limit_az = ""
                pass
        for i in el_list:
            if i < self.soft_limit_down*3600.:
                limit_el = "!!down 1st soft limit!!"
            elif i > self.soft_limit_up*3600.:
                limit_el = "!!up 1st soft limit!!"
            else:
                limit_el = ""
                pass
            limit = limit_az + "" + limit_el
        if limit:
            rospy.logwarn(limit)
        return limit

if __name__ == "__main__":
    rospy.init_node("antenna_server")
    at = antenna()
    at.server_start()
    rospy.spin()
