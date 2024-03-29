#!/usr/bin/env python3

import sys

sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import time
import os
import math
import n_const
import rospy
import threading
from datetime import datetime as dt
from necst.msg import Float64_necst
from necst.msg import String_necst
from necst.msg import Status_antenna_msg
from necst.msg import Status_weather_msg
from necst.msg import Status_encoder_msg
from necst.msg import Status_dome_msg
from necst.msg import Status_drive_msg
from necst.msg import Status_limit_msg
from necst.msg import Read_status_msg
from necst.msg import Bool_necst
from necst.msg import String_list_msg

node_name = "Status"


class status_main(object):
    param1 = {
        "limit_az": 0,
        "limit_el": 0,
        "command_az": 0,
        "command_el": 0,
        "current_az": 0,
        "current_el": 0,
        "emergency": 0,
    }
    param2 = {
        "in_temp": 0,
        "out_temp": 0,
        "in_humi": 0,
        "out_humi": 0,
        "wind_sp": 0,
        "wind_dir": 0,
        "press": 0,
        "rain": 0,
        "cabin_temp1": 0,
        "cabin_temp2": 0,
        "dome_temp1": 0,
        "dome_temp2": 0,
        "gen_temp1": 0,
        "gen_temp2": 0,
    }
    param3 = {"encoder_az": 0, "encoder_el": 0}

    param4 = {
        "move_status": 0,
        "right_act": 0,
        "right_pos": 0,
        "left_act": 0,
        "left_pos": 0,
        "memb_act": 0,
        "memb_pos": "none",
        "remote_status": "none",
        "dome_pos": 0,
        "dome_status": "none",
    }
    param5 = {"position": "none"}
    param6 = {"drive": 0, "contactor": 0}
    param7 = {"position": "none"}
    param8 = {"error": [0] * 32, "error_msg": ""}
    param9 = {"m2_pos": 0}
    param10 = {"alert_msg": ""}
    param11 = {"tracking": False}
    param12 = {"check_launch": ""}

    def __init__(self):
        if __name__ == "__main__":
            th = threading.Thread(target=self.tel_status)
            th.setDaemon(True)
            th.start()
        self.args = sys.argv
        self.args.append("")
        self.pub = rospy.Publisher("read_status", Read_status_msg, queue_size=1)
        pass

    def status_check(self):
        time.sleep(0.1)
        # rospy.loginfo(self.param1)
        # rospy.loginfo("\n")
        # rospy.loginfo(self.param2["rain"])
        # rospy.loginfo("\n")
        # rospy.loginfo(self.param3)
        # rospy.loginfo(self.param4)
        # rospy.loginfo(self.param5)
        # rospy.loginfo(self.param6)
        # rospy.loginfo(self.param7)

    def callback1(self, req):
        self.param1["limit_az"] = req.limit_az
        self.param1["limit_el"] = req.limit_el
        self.param1["command_az"] = req.command_az / 3600.0
        self.param1["command_el"] = req.command_el / 3600.0
        # self.param1["current_az"] = req.current_az
        # self.param1["current_el"] = req.current_el
        self.param1["emergency"] = req.emergency
        pass

    def callback2(self, req):
        self.param2["in_temp"] = req.in_temp
        self.param2["out_temp"] = req.out_temp
        self.param2["in_humi"] = req.in_humi
        self.param2["out_humi"] = req.out_humi
        self.param2["wind_sp"] = req.wind_sp
        self.param2["wind_dir"] = req.wind_dir
        self.param2["press"] = req.press
        self.param2["rain"] = req.rain
        self.param2["cabin_temp1"] = req.cabin_temp1
        self.param2["cabin_temp2"] = req.cabin_temp2
        self.param2["dome_temp1"] = req.dome_temp1
        self.param2["dome_temp2"] = req.dome_temp2
        self.param2["gen_temp1"] = req.gen_temp1
        self.param2["gen_temp2"] = req.gen_temp2
        pass

    def callback3(self, req):
        self.param3["encoder_az"] = req.enc_az / 3600.0
        self.param3["encoder_el"] = req.enc_el / 3600.0
        pass

    def callback4(self, req):
        # status_box = req.status
        # print(status_box)
        self.param4["move_status"] = req.move_status
        self.param4["right_act"] = req.right_act
        self.param4["right_pos"] = req.right_pos
        self.param4["left_act"] = req.left_act
        self.param4["left_pos"] = req.left_pos
        self.param4["memb_act"] = req.memb_act
        self.param4["memb_pos"] = req.memb_pos
        self.param4["remote_status"] = req.remote_status
        dome_pos_1 = float(req.dome_enc)
        # print(status_box[8])
        dome_pos_2 = math.fabs(dome_pos_1) % 1296000
        self.param4["dome_pos"] = math.copysign(dome_pos_2, dome_pos_1) / 3600.0
        # self.param4['dome_pos'] = str(self.param4['dome_pos'])
        if self.param4["right_pos"] == "OPEN" and self.param4["left_pos"] == "OPEN":
            self.param4["dome_status"] = "OPEN"
        elif self.param4["right_pos"] == "MOVE" or self.param4["left_pos"] == "MOVE":
            self.param4["dome_status"] = "MOVE"
        elif self.param4["right_pos"] == "CLOSE" and self.param4["left_pos"] == "CLOSE":
            self.param4["dome_status"] = "CLOSE"
        else:  # Need check
            self.param4["dome_status"] = "ERROR"
        pass

    def callback5(self, req):  # hot
        self.param5["position"] = req.data
        pass

    def callback6(self, req):
        print(req)
        self.param6["drive"] = req.value[0]
        self.param6["contactor"] = req.value[1]
        pass

    def callback7(self, req):
        self.param7["position"] = req.data
        pass

    def callback8(self, req):
        self.param8["error"] = req.error_box
        self.param8["error_msg"] = req.error_msg
        pass

    def callback9(self, req):
        self.param9["m2_pos"] = req.data
        pass

    def callback10(self, req):
        self.param10["alert_msg"] = req.data
        pass

    def callback11(self, req):
        self.param11["tracking"] = req.data
        pass

    def callback12(self, req):
        self.param12["check_launch"] = req.data
        pass

    def tel_status(self):
        print("*********************************")
        print("    NANTEN2 telescope status     ")
        print("*********************************")
        time.sleep(1)

        while 1:
            ###save tel status log config
            now = dt.now()
            dir_name = "/home/amigos/log/{}/{}/{}".format(now.year, now.month, now.day)
            if not os.path.exists(dir_name):
                os.makedirs(dir_name, exist_ok=True)
            f = open(dir_name + "/tel_status.txt", "a")

            # drive = self.param6["drive"]
            enc_az = self.param3["encoder_az"]
            enc_el = self.param3["encoder_el"]
            command_az = self.param1["command_az"]
            command_el = self.param1["command_el"]
            doom_door = self.param4["dome_status"]
            memb_status = self.param4["memb_pos"]
            dome_enc = self.param4["dome_pos"]
            remote_status = self.param4["remote_status"]
            hot_position = self.param5["position"]
            m4_position = self.param7["position"]
            m2_position = self.param9["m2_pos"]
            drive = self.param8["error"][0:4]
            if self.param8["error"][26] == 1:
                antenna_status = "LOCAL"
            else:
                antenna_status = "REMOTE"
            tv = time.time()
            mjd = tv / 24.0 / 3600.0 + 40587.0  # 40587.0 = MJD0

            ntime = dt.now()
            secofday = (
                ntime.hour * 60 * 60
                + ntime.minute * 60
                + ntime.second
                + ntime.microsecond * 0.000001
            )

            lst_g = 0.67239 + 1.00273781 * (mjd - 40000.0)
            l_plb = n_const.LOC_NANTEN2.lon.deg / 360.0
            lst_plb = lst_g + l_plb
            lst_plb_i = int(lst_plb)
            lst_plb -= lst_plb_i
            lst_plb = 24.0 * lst_plb
            lst_hh = int(lst_plb)
            lst_plb = 60.0 * (lst_plb - lst_hh)
            lst_mm = int(lst_plb)
            lst_plb = 60.0 * (lst_plb - lst_mm)
            lst_ss = int(lst_plb)
            lst_hh = "{0:02d}".format(lst_hh)
            lst_mm = "{0:02d}".format(lst_mm)
            lst_ss = "{0:02d}".format(lst_ss)
            log = (
                "telescope: %s %s %s %s %s %5.0f %6.1f %s:%s:%s %5.2f %5.2f  dome: door %s  membrane: %s %s %5.2f HOT :%s M4 :%s"
                % (
                    drive[0],
                    drive[1],
                    drive[2],
                    drive[3],
                    antenna_status,
                    mjd,
                    secofday,
                    lst_hh,
                    lst_mm,
                    lst_ss,
                    enc_az,
                    enc_el,
                    doom_door,
                    memb_status,
                    remote_status,
                    dome_enc,
                    hot_position,
                    m4_position,
                )
            )
            log_debug = (
                "telescope: %s %s %s %s %s %5.0f %6.1f %s:%s:%s %5.2f %5.2f %5.2f %5.2f dome: door %s  membrane: %s %s %5.2f HOT :%s M4 :%s M2 :%s"
                % (
                    drive[0],
                    drive[1],
                    drive[2],
                    drive[3],
                    antenna_status,
                    mjd,
                    secofday,
                    lst_hh,
                    lst_mm,
                    lst_ss,
                    enc_az,
                    enc_el,
                    command_az,
                    command_el,
                    doom_door,
                    memb_status,
                    remote_status,
                    dome_enc,
                    hot_position,
                    m4_position,
                    m2_position,
                )
            )
            f.write(log_debug + "\n")
            f.close()

            print(log_debug)
            if self.param8["error_msg"]:
                print(self.param8["error_msg"])
            if self.param10["alert_msg"]:
                print(self.param10["alert_msg"])
            if self.param12["check_launch"]:
                print("Not moving in the launch file : ", self.param12["check_launch"])

            if self.args[1]:
                if drive[0] == 1:
                    drive = "on"
                else:
                    drive = "off"
                lst = int(lst_hh) * 3600 + int(lst_mm) * 60 + int(lst_ss)
                try:
                    self.pub.publish(
                        Time=tv,
                        Limit=self.param8["error_msg"],
                        Current_Az=enc_az,
                        Current_El=enc_el,
                        Command_Az=command_az,
                        Command_El=command_el,
                        Deviation_Az=command_az - enc_az,
                        Deviation_El=command_el - enc_el,
                        Drive_ready_Az=drive,
                        Drive_ready_El=drive,
                        Authority=antenna_status,
                        Current_Dome=dome_enc,
                        Door_Dome=doom_door,
                        Door_Membrane=memb_status,
                        Door_Authority=remote_status,
                        Current_M4=m4_position,
                        Current_Hot=hot_position,
                        Year=float(ntime.strftime("%Y")),
                        Month=float(ntime.strftime("%m")),
                        Day=float(ntime.strftime("%d")),
                        Hour=float(ntime.strftime("%H")),
                        Min=float(ntime.strftime("%M")),
                        Sec=float(ntime.strftime("%S")),
                        InTemp=self.param2["in_temp"],
                        OutTemp=self.param2["out_temp"],
                        InHumi=self.param2["in_humi"],
                        OutHumi=self.param2["out_humi"],
                        WindDir=self.param2["wind_dir"],
                        WindSp=self.param2["wind_sp"],
                        Press=self.param2["press"],
                        Rain=self.param2["rain"],
                        CabinTemp1=self.param2["cabin_temp1"],
                        CabinTemp2=self.param2["cabin_temp2"],
                        DomeTemp1=self.param2["dome_temp1"],
                        DomeTemp2=self.param2["dome_temp2"],
                        GenTemp1=self.param2["gen_temp1"],
                        GenTemp2=self.param2["gen_temp2"],
                        Current_M2=m2_position,
                        MJD=mjd,
                        LST=lst,
                        Secofday=secofday,
                        from_node=node_name,
                        timestamp=time.time(),
                    )
                except Exception as e:
                    print("error {0}".format(e))
                    print("Propably not running launch.")
            time.sleep(0.05)


class read_status(status_main):
    def __init__(self):
        # status_main.__init__(self)
        # rospy.init_node('Status_read', anonymous=True)
        th = threading.Thread(target=self.initialize)
        th.setDaemon(True)
        th.start()

    def initialize(self):
        sub1 = rospy.Subscriber("status_antenna", Status_antenna_msg, self.callback1)
        sub2 = rospy.Subscriber("status_weather", Status_weather_msg, self.callback2)
        sub3 = rospy.Subscriber("status_encoder", Status_encoder_msg, self.callback3)
        sub4 = rospy.Subscriber("status_dome", Status_dome_msg, self.callback4)
        sub5 = rospy.Subscriber("status_hot", String_necst, self.callback5)
        sub6 = rospy.Subscriber("status_drive", Status_drive_msg, self.callback6)
        sub7 = rospy.Subscriber("status_m4", String_necst, self.callback7)
        sub8 = rospy.Subscriber("limit_check", Status_limit_msg, self.callback8)
        sub9 = rospy.Subscriber("status_m2", Float64_necst, self.callback9)
        sub10 = rospy.Subscriber("alert", String_necst, self.callback10)
        sub11 = rospy.Subscriber("tracking_check", Bool_necst, self.callback11)
        rospy.spin()

    def read_status(self):
        return (
            self.param1,
            self.param2,
            self.param3,
            self.param4,
            self.param5,
            self.param6,
            self.param7,
            self.param8,
            self.param9,
            self.param10,
            self.param11,
        )


if __name__ == "__main__":
    st = status_main()
    rospy.init_node(node_name)
    sub1 = rospy.Subscriber("status_antenna", Status_antenna_msg, st.callback1)
    sub2 = rospy.Subscriber("status_weather", Status_weather_msg, st.callback2)
    sub3 = rospy.Subscriber("status_encoder", Status_encoder_msg, st.callback3)
    sub4 = rospy.Subscriber("status_dome", Status_dome_msg, st.callback4)
    sub5 = rospy.Subscriber("status_hot", String_necst, st.callback5)
    sub6 = rospy.Subscriber("status_drive", Status_drive_msg, st.callback6)
    sub7 = rospy.Subscriber("status_m4", String_necst, st.callback7)
    sub8 = rospy.Subscriber("limit_check", Status_limit_msg, st.callback8)
    sub9 = rospy.Subscriber("status_m2", Float64_necst, st.callback9)
    sub10 = rospy.Subscriber("alert", String_necst, st.callback10)
    sub11 = rospy.Subscriber("tracking_check", Bool_necst, st.callback11)
    sub12 = rospy.Subscriber("check_launch", String_list_msg, st.callback12)
    print("Subscribe Start")
    rospy.spin()
