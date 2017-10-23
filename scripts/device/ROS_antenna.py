#!/usr/bin/env python

import rospy
from necst.msg import Velocity_mode_msg
from necst.msg import Move_mode_msg
from necst.msg import Otf_mode_msg
from necst.msg import Status_encoder_msg
from necst.msg import Status_weather_msg
from necst.msg import list_azelmsg
from datetime import datetime as dt
import sys
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
    temp = ""
    press = ""
    humi = ""
    def __init__(self):
        self.calc = azel_calc.azel_calc()
        import otf
        self.otf = otf.otf()

    def note_encoder(self, req):
        self.enc_az = req.enc_az
        self.enc_el = req.enc_el
        return

    def note_weather(self, req):
        self.temp = req.out_temp #[deg_C]
        self.press = req.press/100. #[Pa]
        self.humi = req.out_humi/100. #[%]
    
    def azel_publish(self, az_list, el_list, start_time):
        pub = rospy.Publisher("list_azel", list_azelmsg, queue_size = 10, latch=True)
        msg = list_azelmsg()
        msg.az_list = az_list
        msg.el_list = el_list
        msg.start_time = start_time
        pub.publish(msg)
        rospy.loginfo('Publish ok.')
        return

    def velocity_move(self, req):
        ret = self.calc.velocity_calc(req.az_speed, req.el_speed, req.dist, self.enc_az, self.enc_el)
        self.azel_publish(ret[0], ret[1], ret[2])
        return

    def radec_move(self, req):
        now = dt.utcnow()
        ret = self.calc.coordinate_calc(req.x, req.y, req.ntarg, req.code_mode,
                                        req.off_x/3600., req.off_y/3600.,
                                        req.offcoord, req.hosei, req.lamda,
                                        req.dcos, self.temp, self.press, self.humi,
                                        now)
        self.azel_publish(ret[0], ret[1], ret[2])
        return

    def galactic_move(self, req):
        now = dt.utcnow()
        ret = self.calc.coordinate_calc(req.x, req.y, req.ntarg, req.code_mode,
                                        req.off_x/3600., req.off_y/3600.,
                                        req.offcoord, req.hosei, req.lamda,
                                        req.dcos, self.temp, self.press, self.humi,
                                        now)
        self.azel_publish(ret[0], ret[1], ret[2])
        return
    
    def planet_move(self, req):
        now = dt.now()
        ret = self.calc.coordinate_calc(req.x, req.y, req.ntarg, req.code_mode,
                                        req.off_x/3600., req.off_y/3600.,
                                        req.offcoord, req.hosei, req.lamda, req.dcos,
                                        self.temp, self.press, self.humi, now)
        self.azel_publish(ret[0], ret[1], ret[2])
        return

    def otf_start(self, req):
        ret = self.otf.otf_scan(req.x, req.y, req.dcos, req.coord_sys, 
                                req.dx, req.dy, req.dt, req.num, req.rampt,
                                req.delay, req.lamda, req.hosei, req.code_mode,
                                req.off_x, req.off_y, req.offcoord,
                                 self.temp, self.press, self.humi)
        self.azel_publish(ret[0], ret[1], ret[2])
        return


if __name__ == "__main__":    
    rospy.init_node("antenna_server")
    rospy.loginfo(" Read ok ")
    at = antenna()
    rospy.Subscriber('antenna_vel', Velocity_mode_msg, at.velocity_move)
    rospy.Subscriber('antenna_radec', Move_mode_msg, at.radec_move)
    rospy.Subscriber('antenna_galactic', Move_mode_msg, at.galactic_move)
    rospy.Subscriber('antenna_planet', Move_mode_msg, at.planet_move)
    rospy.Subscriber('antenna_otf', Otf_mode_msg, at.otf_start)
    rospy.Subscriber("status_encoder", Status_encoder_msg, at.note_encoder)
    rospy.Subscriber('status_weather', Status_weather_msg, at.note_weather)
    rospy.spin()
