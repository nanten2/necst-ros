#! /usr/bin/env python
# coding:utf-8

"""
------------------------------------------------
[History]
2017/09/15 : kondo takashi
------------------------------------------------
"""

import time
from datetime import datetime as dt
import sys
import rospy
#from ros_start.srv import NECSTsrv
#from ros_start.srv import NECSTsrvResponse
from ros_start.msg import drive_msg
from ros_start.msg import Velocity_mode_msg
from ros_start.msg import Move_mode_msg
from ros_start.msg import Otf_mode_msg
#from ros_start.msg import dome_drive_msg
#from ros_start.msg import dome_move_msg
#from ros_start.msg import membrane_msg
from ros_start.msg import NECST_msg
from ros_start.msg import Dome_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float64

class controller(object):

    task_flag = False
    tracking_flag = False
    access_authority = "no_data"
    

    def __init__(self):
        rospy.init_node('controller_client')
        rospy.Subscriber("error", Bool, self.error_check)
        rospy.Subscriber("task_check", Bool, self.antenna_flag)
        rospy.Subscriber("tracking_check", Bool, self.tracking_flag)
        rospy.Subscriber("authority_check", String, self.authority_check)
        #pub = rospy.Publisher("stop", String, queue_size = 10)
        #rospy.spin()

        return
    
    def authority_check(self, req):
        self.access_authority = req.data
        return


    def authority(self, user = "release"):
        while self.access_authority == "no_data":
            print("wait")
            time.sleep(1)
        if self.access_authority == "release" and user == "release":
            rospy.loginfo("Already release...")
        elif user == "release":
            rospy.loginfo("release authority")
        elif self.access_authority == "release":
            rospy.loginfo("Authority_change.")
        elif self.access_authority == user:
            rospy.loginfo("Authority is already you.")
        else:
            rospy.loginfo("Authority is other!!")
            #sys.exit()
        pub = rospy.Publisher("authority_change", String, queue_size = 10,latch=True)
        msg = String()
        msg.data = user
        pub.publish(msg)
        time.sleep(0.01)

    def observation(self, command, exposure):
        pub = rospy.Publisher("getting_data", Bool, queue_size=10)
        msg = Bool()
        if command == "start":
            msg.data = True
            #msg.data2 = exposure
        elif command == "end":
            msg.data = False
        else:
            rospy.logerr("argument is error!!")
            sys.exit()
        pub.publish(msg)
        return

    def spectrometer(self, exposure):
        pub = rospy.Publisher("getting_data", Float64, queue_size=10)
        msg = Float64()
        msg.data = exposure
        pub.publish(msg)
        return

    def error_check(self, req):
        # error --> True ... stop
        #if not self.error:
        if not req.data:
            pass
        elif req.data:
            rospy.logerr("Error stop !!\n")
            print("Error stop !!\n")
            rospy.signal_shutdown("Error stop !!\n")
        else:
            rospy.logerr("Command error !!\n")
            rospy.logerr("Please check script.\n")
            print("Command error !!\n")
            print("Please check script.\n")
            rospy.signal_shutdown("Error stop !!\n")
        return

    def antenna_flag(self, req):
        self.task_flag = req.data 
        return

    def antenna_check(self):
        while self.task_flag:
            rospy.loginfo(" move now... \n")
            print(" move now... \n")
            time.sleep(0.01)
            pass
        return

    def tracking_flag(self, req):
        self.tracking_flag = req.data
        return

    def tracking_check(self):
        while not self.tracking_flag:
            rospy.loginfo(" tracking now... \n")
            print(" tracking now... \n")
            time.sleep(0.01)
            pass

    def emergency(self):#shiotani added 09/25
        pub = rospy.Publisher('emergency', Bool, queue_size = 10, latch = True)
        emergen_call = Bool()
        emergen_call.data = True
        pub.publish(emergen_call)
        rospy.logwarn('!!!emergency called ROS_control.py!!!')
        


    def drive_on(self):
        """drive_on"""
        pub = rospy.Publisher("antenna_drive", String, queue_size = 10)
        msg = String()
        msg.data = "on"
        pub.publish(msg) 
        return

    def drive_off(self):
        """drive_off"""
        pub = rospy.Publisher("antenna_drive", String, queue_size = 10)
        msg = String()
        msg.data = "off"
        pub.publish(msg)
        return

    def contactor_on(self):
        pub = rospy.Publisher("antenna_contactor", String, queue_size = 10)
        msg = String()
        msg.data = "on"
        pub.publish(msg)
        return

    def contactor_off(self):
        pub = rospy.Publisher("antenna_contactor", String, queue_size = 10)
        msg = String()
        msg.data = "off"
        pub.publish(msg)
        return

    def velocity_move(self, az_speed, el_speed, dist_arcsec = 5 * 3600):
        pub = rospy.Publisher("antenna_vel", Velocity_mode_msg, queue_size = 10, latch = True)
        vel = Velocity_mode_msg()
        vel.az_speed = az_speed
        vel.el_speed = el_speed
        vel.dist = dist_arcsec
        pub.publish(vel)
        rospy.loginfo(vel)
        return

    def radec_move(self, ra, dec, code_mode, off_x = 0, off_y = 0, offcoord = 'HORIZONTAL', hosei = 'hosei_230.txt',  lamda=2600, dcos=0, az_rate=12000, el_rate=12000,):
        #self.ant.radec_move(ra, dec, code_mode, off_x, off_y, hosei, offcoord, lamda, az_rate, el_rate, dcos)
        pub = rospy.Publisher("antenna_radec", Move_mode_msg, queue_size = 10, latch = True)
        msg = Move_mode_msg()
        msg.x = ra
        msg.y = dec
        msg.code_mode = code_mode
        msg.off_x = off_x
        msg.off_y = off_y
        msg.hosei = hosei
        msg.offcoord = offcoord
        msg.lamda = lamda
        msg.dcos = dcos
        #mv.az_rate ... no inplementation
        #mv.el_rate ... no inplementation
        rospy.loginfo(msg)
        pub.publish(msg)
        return
    

    def galactic_move(self, l, b, off_x = 0, off_y = 0, offcoord = 'HORIZONTAL', hosei = 'hosei_230.txt', lamda=2600, az_rate=12000, el_rate=12000, dcos=0):
        pub = rospy.Publisher("antenna_galactic", Move_mode_msg, queue_size = 10, latch = True)
        msg = Move_mode_msg()
        msg.x = l
        msg.y = b
        msg.off_x = off_x
        msg.off_y = off_y
        msg.code_mode = "galactic"
        msg.hosei = hosei
        msg.offcoord = offcoord
        msg.lamda = lamda
        msg.dcos = dcos
        #mv.az_rate ... no inplementation
        #mv.el_rate ... no inplementation
        rospy.loginfo(msg)
        pub.publish(msg)
        return

    def planet_move(self, number, off_x = 0, off_y = 0, offcoord = 'HORIZONTAL', hosei = 'hosei_230.txt', lamda=2600, az_rate=12000, el_rate=12000, dcos=0):
        """1.Mercury 2.Venus 3. 4.Mars 5.Jupiter 6.Saturn 7.Uranus 8.Neptune, 9.Pluto, 10.Moon, 11.Sun"""
        pub = rospy.Publisher("antenna_planet", Move_mode_msg, queue_size = 10, latch = True)
        msg = Move_mode_msg()
        msg.ntarg = number
        msg.off_x = off_x
        msg.off_y = off_y
        msg.code_mode = "planet"
        msg.hosei = hosei
        msg.offcoord = offcoord
        msg.lamda = lamda
        msg.dcos = dcos
        #mv.az_rate ... no inplementation
        #mv.el_rate ... no inplementation
        rospy.loginfo(msg)
        pub.publish(msg)
        return

    def move_stop(self):
        pub = rospy.Publisher("move_stop", String, queue_size = 10, latch = True)
        msg = String()
        msg.data = "stop"
        print("move_stop")
        pub.publish(msg)
        return
        

    def otf_scan(self, lambda_on, beta_on, coord_sys, dx, dy, dt, num, rampt, delay, lamda, hosei, code_mode, off_x, off_y, off_coord, dcos=0, ntarg = 0):
        #on_start = self.ant.otf_start(lambda_on, beta_on, dcos, coord_sys, dx, dy, dt, num, rampt, delay, lamda, hosei, code_mode, off_x, off_y, off_coord, ntarg)
        pub = rospy.Publisher("antenna_otf", Otf_mode_msg, queue_size = 10, latch = True)
        msg = Otf_mode_msg()
        msg.x = lambda_on
        msg.y = beta_on
        msg.code_mode = code_mode
        msg.off_x = off_x
        msg.off_y = off_y
        msg.hosei = hosei
        msg.offcoord = off_coord
        msg.lamda = lamda
        msg.dcos = dcos
        msg.ntarg = ntarg
        msg.dx = dx
        msg.dy = dy
        msg.dt = dt
        msg.num = num
        msg.rampt = rampt
        msg.delay = delay
        rospy.loginfo(msg)
        pub.publish(msg)
        
        return 

    def read_track(self):
        ret = self.ant.read_track()
        if ret[0] == "TRUE" and ret[1] == "TRUE":
            flag = True
        else:
            flag = False
        return flag


    def dome_move(self,dist):
        pub = rospy.Publisher("dome_move", Dome_msg, queue_size = 10, latch = True)
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_move'
        pub.publish(dome)
        dome = Dome_msg()
        dome.name = 'target_az'
        dome.value = str(dist)
        pub.publish(dome)

    
    def dome_open(self):
        #"""Dome\u306eopen"""
        #self.ant.dome_open()
        #return
        pub = rospy.Publisher("dome_move", Dome_msg, queue_size = 10, latch = True)
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_open'
        pub.publish(dome)
    
    def dome_close(self):
        #"""Dome\u306eclose"""
        #self.ant.dome_close()
        #return
        pub = rospy.Publisher("dome_move", Dome_msg, queue_size = 10, latch = True)
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_close'
        pub.publish(dome)
        
    def memb_open(self):
        """\u30e1\u30f3\u30d6\u30ec\u30f3\u306eopen"""
        #self.ant.memb_open()
        #return
        pub = rospy.Publisher("dome_move", Dome_msg, queue_size = 10, latch = True)
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'memb_open'
        pub.publish(dome)

    def memb_close(self):
        #"""\u30e1\u30f3\u30d6\u30ec\u30f3\u306eopenclose"""
        #self.ant.memb_close()
        #return
        pub = rospy.Publisher("dome_move", Dome_msg, queue_size = 10, latch = True)
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'memb_close'
        pub.publish(dome)
        
    #def dome_move(self, dome_az):
       # """Dome\u3092(dome_az)\u306b\u52d5\u4f5c"""
        #self.ant.dome_move(dome_az)
        #return

    def dome_stop(self):
        """Dome\u306eclose\u52d5\u4f5c\u3092\u505c\u6b62"""
        #self.dome_track_end()
        #self.ant.dome_stop()
        #return
        pub = rospy.Publisher("dome_move", Dome_msg, queue_size = 10, latch = True)
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_stop'
        pub.publish(dome)
        
    def dome_track(self):
        """Dome\u3068\u671b\u9060\u93e1\u306esync"""
        #self.ant.dome_track()
        #return
        pub = rospy.Publisher("dome_move", Dome_msg, queue_size = 10, latch = True)
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_tracking'
        pub.publish(dome)

    def dome_track_end(self):
        """Dome\u3068\u671b\u9060\u93e1\u306esync\u306e\u7d42\u4e86"""
        #self.ant.dome_track_end()
        #return
        pub = rospy.Publisher("dome_move", Dome_msg, queue_size = 10, latch = True)
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_track_end'
        pub.publish(dome)

    def dome_tracking_check(self):
        time.sleep(2)
        return True

# ===================
# mirror
# ===================

    def move_m4(self, position):
        """mirror4\u3092\u52d5\u304b\u3059("in"or"out")"""
        pub = rospy.Publisher('m4', String, queue_size = 10, latch = True)
        status = String()
        status.data = position
        pub.publish(status)
        return

    
    def move_hot(self, position):
        """hotload\u3092\u52d5\u304b\u3059("in"or"out")"""
        #if position == "in": self.beam.hot_in()
        #elif position == "out": self.beam.hot_out()
        #else : print('set hotposition error')
        #return
        pub = rospy.Publisher("hot", String, queue_size = 10, latch = True)
        status = String()
        status.data = position
        pub.publish(status)
        return

    def m2_move(self, dist):
        """m2\u3092\u52d5\u304b\u3059(um)"""
        self.beam.m2_move(dist)
        return

# ===================
# encoder
# ===================

    def oneshot(self, repeat=1, exposure=1.0, stime=0.0):
        #\u5206\u5149\u8a08oneshot\u306ecount\u5024\u3092\u914d\u5217\u3067\u51fa\u529b
        #dfs01 = self.rx.oneshot_dfs01(repeat, exposure ,stime)
        #dfs02 = self.rx.oneshot_dfs02(repeat, exposure ,stime)
        data = self.rx.oneshot_dfs(repeat, exposure, stime)
        data_dict = {'dfs1': data[0], 'dfs2': data[1]}
        return data_dict

# ===================
# status
# ===================

    def read_status(self):
        """\u6a5f\u5668and\u5929\u6c17\u306e\u30b9\u30c6\u30fc\u30bf\u30b9\u3092\u53d6\u5f97_"""
        timestamp = time.strftime('%Y/%m/%d %H:%M:%S',time.gmtime())
        ant_status = self.status.read_antenna()
        beam_status = self.status.read_beam()
        # sg_status = self.doppler.get_status()
        ret = self.status.read_weather()
        
        tv = time.time()
        mjd = tv/24./3600. + 40587.0
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
        lst = 100
        
        if ant_status[1][0] & ant_status[1][1] == 1:
            drive_ready_az = 'ON'
        else:
            drive_ready_az = 'OFF'

        if ant_status[1][2] & ant_status[1][3] == 1:
            drive_ready_el = 'ON'
        else:
            drive_ready_el = 'OFF'

        if ant_status[1][24] == 1:
            emergency = 'ON'
        else:
            emergency = 'OFF'

        if ant_status[5][1][1] == 'OPEN' and ant_status[5][1][3] == 'OPEN':
            door_dome = 'OPEN'
        elif ant_status[5][1][1] == 'MOVE' or ant_status[5][1][3] == 'MOVE':
            door_dome = 'MOVE'
        elif ant_status[5][1][1] == 'CLOSE' and ant_status[5][1][3] == 'CLOSE':
            door_dome = 'CLOSE'
        else:
            door_dome = 'ERROR'

        statusbox = { "Time" : timestamp,
                   "Limit" : ant_status[0],
                   "Current_Az" : ant_status[4][0]/3600.,
                   "Current_El" : ant_status[4][1]/3600.,
                   "Command_Az" : ant_status[3][2]/3600.,
                   "Command_El" : ant_status[3][3]/3600.,
                   "Deviation_Az" : ant_status[3][4],
                   "Deviation_El" : ant_status[3][5],
                   "Drive_ready_Az" : drive_ready_az,
                   "Drive_ready_El": drive_ready_el,
                   "Authority" : ant_status[2],
                   "Emergency" : emergency,
                   "Current_Dome" : ant_status[6]/3600.,
                   "Door_Dome" : door_dome,
                   "Door_Membrane" : ant_status[5][2][1],
                   "Door_Authority" : ant_status[5][3],
                   "Current_M4" : beam_status[1],
                   "Current_Hot" : beam_status[0],
                   "Year" : ret[0],
                   "Month" : ret[1],
                   "Day" : ret[2],
                   "Hour" : ret[3],
                   "Min" : ret[4],
                   "Sec" : ret[5],
                   "InTemp" : ret[6],
                   "OutTemp" : ret[7],
                   "InHumi" : ret[8],
                   "OutHumi" : ret[9],
                   "WindDir" : ret[10],
                   "WindSp" : ret[11],
                   "Press" : ret[12],
                   "Rain" : ret[13],
                   "CabinTemp1" : ret[14],
                   "CabinTemp2" :ret[15],
                   "DomeTemp1" : ret[16],
                   "DomeTemp2" : ret[17],
                   "GenTemp1" : ret[18],
                   "GenTemp2" : ret[19],
                   "None" : 'None',
                   "Current_M2" : beam_status[2],
                   "MJD" : int(mjd),
                   "LST" : lst,
                   "Secofday" : secofday
                   }
                   
        return statusbox

if __name__ == "__main__":
    con = controller()

    # test
    aa = str(input("Please input mode (j2000, b1950, gal, planet, vel) : "))
    if aa == "j2000" or aa == "":
        con.radec_move(83, -5, "J2000")
    elif aa == "b1950":
        con.radec_move(28, 34, "b1950")
    elif aa == "gal":
        con.galactic_move(28, 34)
    elif aa == "planet":
        con.planet_move(2)
    elif aa == "vel":
        con.velocity_move(-2000, 0, 10*3600)
    elif isinstance(aa ,str):
        bb = input()
        con.otf_scan(float(aa), float(bb), 1, "j2000", 30, 0, 0.6, 9, 4*0.6, 0, lamda=2600, hosei="hosei_230.txt", code_mode="j2000", off_x=-900, off_y=-900, off_coord="j2000", ntarg = 0)
    else:
        print("no name")
    time.sleep(0.1)
    #self, lambda_on, beta_on, dcos, coord_sys, dx, dy, dt, num, rampt, delay, lamda, hosei, code_mode, off_x, off_y, off_coord, ntarg = 0
  

