#!/usr/bin/env python3

import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import os
import time
import math
import rospy
import threading
from datetime import datetime as dt
from std_msgs.msg import Float64
from std_msgs.msg import String
from necst.msg import Status_antenna_msg
from necst.msg import Status_weather_msg
from necst.msg import Status_encoder_msg
from necst.msg import Status_dome_msg
from necst.msg import Status_hot_msg
from necst.msg import Status_drive_msg
from necst.msg import Status_m4_msg
from necst.msg import Status_limit_msg

# --                                                                            
data_exp_dir = '/home/amigos/data/experiment'
node_name = 'save_telescope_status'
home_dir = os.path.join(data_exp_dir, node_name)
# --       

class status_main(object):
    param1 = {
        "limit_az":0,
        "limit_el":0,
        "command_az":0,
        "command_el":0,
        "current_az":0,
        "current_el":0,
        "emergency" :0
        }
    param2 = {"in_temp":0,
              "out_temp":0,
              "in_humi":0,         
              "out_humi":0,
              "wind_sp":0,
              "wind_dir":0,
              "press":0,
              "rain":0,
              "cabin_temp1":0,
              "cabin_temp2":0,
              "dome_temp1":0,
              "dome_temp2":0,
              "gen_temp1":0,
              "gen_temp2":0,
              }
    param3 = {"encoder_az":0,
              "encoder_el":0
              }

    param4 = {'move_status':0,
              'right_act':0,
              'right_pos':0,
              'left_act':0,
              'left_pos':0,
              'memb_act':0,
              'memb_pos':0,
              'remote_status':0,
              'dome_pos':0,
              'dome_status':0
              }
    param5 = {'position':'none'}
    param6 = {"drive":0,
              "contactor":0
              }
    param7 = {'position':'none'}
    param8 = {"error":[0]*32,
              "error_msg":""}
    param9 = {"m2_pos": 0}
    param10 = {"alert_msg":""}

    def __init__(self):
        th = threading.Thread(target = self.tel_status)
        th.setDaemon(True)
        th.start()
        pass

    def status_check(self):
        time.sleep(0.1)
        #rospy.loginfo(self.param1)
        #rospy.loginfo("\n")
        #rospy.loginfo(self.param2["rain"])
        #rospy.loginfo("\n")
        #rospy.loginfo(self.param3)
        #rospy.loginfo(self.param4)
        #rospy.loginfo(self.param5)
        #rospy.loginfo(self.param6)
        #rospy.loginfo(self.param7)

    def callback1(self, req):
        self.param1["limit_az"] = req.limit_az
        self.param1["limit_el"] = req.limit_el
        self.param1["command_az"] = req.command_az/3600.
        self.param1["command_el"] = req.command_el/3600.
        #self.param1["current_az"] = req.current_az
        #self.param1["current_el"] = req.current_el
        self.param1["emergency"] = req.emergency
        self.status_check()

    def callback2(self, req):
        self.param2["in_temp"] = req.in_temp
        self.param2["out_temp"]= req.out_temp
        self.param2["in_humi"]= req.in_humi
        self.param2["out_humi"]= req.out_humi
        self.param2["wind_sp"]= req.wind_sp
        self.param2["wind_dir"]= req.wind_dir
        self.param2["press"]= req.press
        self.param2["rain"]= req.rain
        self.param2["cabin_temp1"]= req.cabin_temp1
        self.param2["cabin_temp2"]= req.cabin_temp2
        self.param2["dome_temp1"]= req.dome_temp1
        self.param2["dome_temp2"]= req.dome_temp2
        self.param2["gen_temp1"]= req.gen_temp1
        self.param2["gen_temp2"]= req.gen_temp2
        self.status_check()
        pass

    def callback3(self, req):
        self.param3["encoder_az"] = req.enc_az/3600.
        self.param3["encoder_el"] = req.enc_el/3600.
        self.status_check()
        pass
    
    def callback4(self, req):
        status_box = req.status
        #print(status_box)
        self.param4['move_status'] = status_box[0]
        self.param4['right_act'] = status_box[1]
        self.param4['right_pos'] = status_box[2]
        self.param4['left_act'] = status_box[3]
        self.param4['left_pos'] = status_box[4]
        self.param4['memb_act'] = status_box[5]
        self.param4['memb_pos'] = status_box[6]
        self.param4['remote_status'] = status_box[7]
        dome_pos_1 = float(req.dome_enc)
        #print(status_box[8])
        dome_pos_2 = math.fabs(dome_pos_1)%1296000
        self.param4['dome_pos'] = math.copysign(dome_pos_2,dome_pos_1)/3600.
        #self.param4['dome_pos'] = str(self.param4['dome_pos'])
        if self.param4['right_pos'] == 'OPEN' and self.param4['left_pos'] == 'OPEN':
            self.param4['dome_status'] = 'OPEN'
        elif self.param4['right_pos'] == 'MOVE' or self.param4['left_pos'] == 'MOVE':
            self.param4['dome_status'] = 'MOVE'
        elif self.param4['right_pos'] == 'CLOSE' and self.param4['left_pos'] == 'CLOSE':
            self.param4['dome_status'] = 'CLOSE'
        self.status_check()
        pass
        
    def callback5(self, req):
        self.param5['position'] = req.hot_position
        self.status_check()
        pass

    def callback6(self, req):
        print(req)
        self.param6["drive"] = req.value[0]
        self.param6["contactor"] = req.value[1]
        self.status_check()
        pass

    def callback7(self,req):
        self.param7['position'] = req.m4_position
        self.status_check()
        pass

    def callback8(self,req):
        self.param8["error"] = req.error_box
        self.param8["error_msg"] = req.error_msg
        self.status_check()
        pass

    def callback9(self,req):
        self.param9["m2_pos"] = req.data
        self.status_check()
        pass

    def callback10(self,req):
        self.param10["alert_msg"] = req.data
        self.status_check()
        pass

    def tel_status(self):
        file_memo = input('input file_name: ')
        ut = time.gmtime()
        filename = time.strftime("%Y_%m_%d_%H_%M_%S_" + file_memo + ".txt", ut)
        saveto = os.path.join(home_dir, filename)
        print('*********************************')
        print('    NANTEN2 telescope status     ')
        print('*********************************')
        time.sleep(1)
        print('save to: ', filename)

        while(1):
            #drive = self.param6["drive"]
            enc_az = self.param3['encoder_az']
            enc_el = self.param3['encoder_el']
            command_az = self.param1['command_az']
            command_el = self.param1['command_el']
            doom_door = self.param4['dome_status']
            memb_status = self.param4['memb_pos']
            dome_enc = self.param4['dome_pos']
            remote_status = self.param4['remote_status']
            hot_position = self.param5['position']
            m4_position = self.param7['position']
            m2_position = self.param9["m2_pos"]
            drive = self.param8["error"][0:4]
            if self.param8["error"][26] == 1:
                antenna_status = "Local"
            else:
                antenna_status = "REMOTE"
            
            tv = time.time()
            mjd = tv/24./3600. + 40587.0 # 40587.0 = MJD0

            ntime = dt.now()
            secofday = ntime.hour*60*60 + ntime.minute*60 + ntime.second + ntime.microsecond*0.000001
    
            lst_g = 0.67239+1.00273781*(mjd-40000.0)
            l_plb = -67.7222222222/360.0
            lst_plb = lst_g + l_plb
            lst_plb_i = int(lst_plb)
            lst_plb -= lst_plb_i
            lst_plb = 24.0*lst_plb
            lst_hh = int(lst_plb)
            lst_plb = 60.0*(lst_plb - lst_hh)
            lst_mm = int(lst_plb)
            lst_plb = 60.0*(lst_plb -lst_mm)
            lst_ss = int(lst_plb)
            lst_hh = "{0:02d}".format(lst_hh)
            lst_mm = "{0:02d}".format(lst_mm)
            lst_ss = "{0:02d}".format(lst_ss)
            log_debug = "telescope: %s %s %s %s %s %5.0f %6.1f %s:%s:%s %5.2f %5.2f %5.2f %5.2f dome: door %s  membrane: %s %s %5.2f HOT: %s M4: %s M2: %s" %(drive[0],drive[1], drive[2], drive[3], antenna_status, mjd, secofday, lst_hh, lst_mm, lst_ss, enc_az, enc_el, command_az, command_el, doom_door, memb_status, remote_status, dome_enc, hot_position, m4_position, m2_position)
            f = open(saveto,'a')
            f.write(log_debug  + "\n")
            print(log_debug)
            if self.param8["error_msg"]:
                print(self.param8["error_msg"])
                f.write(self.param8["error_msg"])
            if self.param10["alert_msg"]:
                print(self.param10["alert_msg"])
                f.write(self.param10["alert_msg"])
            f.close()
            time.sleep(1.)


if __name__ == '__main__':
    if not os.path.exists(home_dir):
        os.makedirs(home_dir)
        pass
    st = status_main()
    rospy.init_node('save_status')
    sub1 = rospy.Subscriber('status_antenna', Status_antenna_msg, st.callback1)
    time.sleep(0.01)
    sub2 = rospy.Subscriber('status_weather', Status_weather_msg, st.callback2)
    time.sleep(0.01)
    sub3 = rospy.Subscriber('status_encoder', Status_encoder_msg, st.callback3)
    sub4 = rospy.Subscriber('status_dome', Status_dome_msg, st.callback4)
    sub5 = rospy.Subscriber('status_hot', Status_hot_msg, st.callback5)
    sub6 = rospy.Subscriber('status_drive', Status_drive_msg, st.callback6)
    sub7 = rospy.Subscriber('status_m4', Status_m4_msg, st.callback7)
    sub8 = rospy.Subscriber('limit_check', Status_limit_msg, st.callback8)
    sub9 = rospy.Subscriber('status_m2', Float64, st.callback9)
    sub10 = rospy.Subscriber('alert', String, st.callback10)
    print("Subscribe Start")
    rospy.spin()
