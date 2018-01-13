#!/usr/bin/env python3

import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import rospy
import time
from std_msgs.msg import String
from necst.msg import Velocity_mode_msg
from necst.msg import Move_mode_msg
from necst.msg import Otf_mode_msg
from necst.msg import Status_encoder_msg
from necst.msg import Status_weather_msg
from necst.msg import list_azelmsg
from datetime import datetime as dt
sys.path.append("/home/necst/ros/src/necst/lib")
sys.path.append("/home/amigos/ros/src/necst/lib")
import azel_calc
import otf


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
    temp = ""
    press = ""
    humi = ""

    limit = 0
    stime = 0
    
    def __init__(self):
        self.calc = azel_calc.azel_calc()
        self.otf = otf.otf()
        self.stime = time.time()

    def note_encoder(self, req):
        self.enc_az = req.enc_az
        self.enc_el = req.enc_el
        return

    def note_weather(self, req):
        self.temp = req.out_temp #[deg_C]
        self.press = req.press/100. #[Pa]
        self.humi = req.out_humi/100. #[%]

    def move_stop(self, req):
        self.stime = time.time()
        
    def azel_publish(self, az_list, el_list, start_time, limit = True, stime = 0):
        pub = rospy.Publisher("list_azel", list_azelmsg, queue_size = 1, latch=True)
        msg = list_azelmsg()
        msg.az_list = az_list
        msg.el_list = el_list
        msg.start_time = start_time
        if limit:
            ret = self.limit_check(az_list, el_list)
        else:
            ret = ""
            pass
        if ret:
            rospy.logerr("Publish False...")
        elif stime < self.stime:
            rospy.logwarn("move stop!!")
        else:
            pub.publish(msg)
            rospy.loginfo('Publish ok.')
            pass
        return

    def velocity_move(self, req):
        ret = self.calc.velocity_calc(req.az_speed, req.el_speed, req.dist, self.enc_az, self.enc_el)
        self.azel_publish(ret[0], ret[1], ret[2], req.limit, req.time)
        return

    def azel_move(self,req):
        now = dt.utcnow()
        print("start calculation")
        ret = self.calc.azel_calc(req.x, req.y, 
                                  req.off_x/3600., req.off_y/3600.,
                                  req.offcoord, now, req.vel_x, req.vel_y)
        print("end calculation")
        self.azel_publish(ret[0], ret[1], ret[2], req.limit, req.time)
        return
        

    def radec_move(self, req):
        now = dt.utcnow()
        print("start calculation")
        ret = self.calc.coordinate_calc(req.x, req.y, req.ntarg, req.code_mode,
                                        req.off_x/3600., req.off_y/3600.,
                                        req.offcoord, req.hosei, req.lamda,
                                        req.dcos, self.temp, self.press, self.humi,
                                        now)
        print("end calculation")
        self.azel_publish(ret[0], ret[1], ret[2], req.limit, req.time)
        return

    def galactic_move(self, req):
        now = dt.utcnow()
        print("start calculation")
        ret = self.calc.coordinate_calc(req.x, req.y, req.ntarg, req.code_mode,
                                        req.off_x/3600., req.off_y/3600.,
                                        req.offcoord, req.hosei, req.lamda,
                                        req.dcos, self.temp, self.press, self.humi,
                                        now)
        print("end calculation")
        self.azel_publish(ret[0], ret[1], ret[2], req.limit, req.time)
        return
    
    def planet_move(self, req):
        now = dt.now()
        print("start calculation")
        ret = self.calc.coordinate_calc(req.x, req.y, req.ntarg, req.code_mode,
                                        req.off_x/3600., req.off_y/3600.,
                                        req.offcoord, req.hosei, req.lamda, req.dcos,
                                        self.temp, self.press, self.humi, now)
        print("end calculation")
        self.azel_publish(ret[0], ret[1], ret[2], req.limit, req.time)
        return

    def otf_start(self, req):
        ret = self.otf.otf_scan(req.x, req.y, req.dcos, req.coord_sys, 
                                req.dx, req.dy, req.dt, req.num, req.rampt,
                                req.delay, req.lamda, req.hosei, req.code_mode,
                                req.off_x, req.off_y, req.offcoord,
                                 self.temp, self.press, self.humi)
        self.azel_publish(ret[0], ret[1], ret[2], req.limit, req.time)
        return

    def limit_check(self, az_list, el_list):
        for i in az_list:
            if i > 240.*3600.:
                limit_az = "!!cw limit!!"
            elif i <-240.*3600.:
                limit_az = "!!ccw limit!!"
            else:
                limit_az = ""
                pass
        for i in el_list:
            if i < 20.*3600.:
                limit_el = "!!down limit!!"
            elif i > 80.*3600.:
                limit_el = "!!up limit!!"
            else:
                limit_el = ""
                pass
            limit = limit_az + "" + limit_el
        rospy.loginfo(limit)
        return limit

if __name__ == "__main__":    
    rospy.init_node("antenna_server")
    rospy.loginfo(" Read ok ")
    at = antenna()
    rospy.Subscriber("status_encoder", Status_encoder_msg, at.note_encoder)
    rospy.Subscriber('status_weather', Status_weather_msg, at.note_weather)
    rospy.Subscriber("move_stop", String, at.move_stop)
    time.sleep(0.1)
    rospy.Subscriber('antenna_vel', Velocity_mode_msg, at.velocity_move)
    rospy.Subscriber('antenna_azel', Move_mode_msg, at.azel_move)
    rospy.Subscriber('assist_radec', Move_mode_msg, at.radec_move)
    rospy.Subscriber('assist_galactic', Move_mode_msg, at.galactic_move)
    rospy.Subscriber('assist_planet', Move_mode_msg, at.planet_move)
    rospy.Subscriber('antenna_otf', Otf_mode_msg, at.otf_start)
    rospy.spin()
