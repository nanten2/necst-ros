#! /usr/bin/env python3
# coding:utf-8

"""
------------------------------------------------
[History]
2017/10/18 : kondo takashi
------------------------------------------------
"""
import sys
import time
from datetime import datetime as dt
import rospy

from necst.msg import drive_msg
from necst.msg import Velocity_mode_msg
from necst.msg import Move_mode_msg
from necst.msg import Otf_mode_msg
from necst.msg import Dome_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int64

class controller(object):

    task_flag = False
    antenna_tracking_flag = False
    dome_tracking_flag = False
    access_authority = "no_data"
    

    def __init__(self):
        rospy.init_node('controller_client')
        rospy.Subscriber("error", Bool, self.error_check)
        #rospy.Subscriber("task_check", Bool, self.antenna_flag)
        rospy.Subscriber("tracking_check", Bool, self.antenna_tracking)
        rospy.Subscriber("dome_tracking_check", Bool, self.dome_tracking)
        rospy.Subscriber("authority_check", String, self.authority_check)

        self.pub1 = rospy.Publisher("observation_start_", Bool, queue_size=10)#observation
        self.pub2 = rospy.Publisher("authority_change", String, queue_size = 10,latch=True)
        self.pub3 = rospy.Publisher("getting_data", Float64, queue_size=10)
        self.pub4 = rospy.Publisher('emergency_stop', Bool, queue_size = 10, latch = True)
        self.pub5 = rospy.Publisher("antenna_drive", String, queue_size = 10)
        self.pub6 = rospy.Publisher("antenna_contactor", String, queue_size = 10)
        self.pub7 = rospy.Publisher("antenna_vel", Velocity_mode_msg, queue_size = 10, latch = True)
        self.pub8 = rospy.Publisher("antenna_radec", Move_mode_msg, queue_size = 10, latch = True)
        self.pub9 = rospy.Publisher("antenna_galactic", Move_mode_msg, queue_size = 10, latch = True)
        self.pub10 = rospy.Publisher("antenna_planet", Move_mode_msg, queue_size = 10, latch = True)
        self.pub11 = rospy.Publisher("move_stop", String, queue_size = 10, latch = True)
        self.pub12 = rospy.Publisher("antenna_otf", Otf_mode_msg, queue_size = 10, latch = True)
        self.pub13 = rospy.Publisher("dome_move", Dome_msg, queue_size = 10, latch = True)
        self.pub14 = rospy.Publisher('m4', String, queue_size = 10, latch = True)
        self.pub15 = rospy.Publisher("hot", String, queue_size = 10, latch = True)
        self.pub16 = rospy.Publisher("m2", Int64, queue_size=10, latch=True)
        self.pub17 = rospy.Publisher("antenna_azel", Move_mode_msg, queue_size=10, latch=True)
        #self.pub18 = rospy.Publisher("oneshot_achilles", Oneshot_achiless_msg, queue_size=10, latch=True )
        time.sleep(0.5)


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
        msg = String()
        msg.data = user
        self.pub2.publish(msg)
        time.sleep(0.01)

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

    """
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
        """

    def antenna_tracking(self, req):
        self.antenna_tracking_flag = req.data
        return

    def antenna_tracking_check(self):
        rospy.loginfo(" tracking now... \n")
        while not self.antenna_tracking_flag:
            time.sleep(0.01)
            pass

    def emergency(self):#shiotani added 09/25
        emergen_call = Bool()
        emergen_call.data = True
        self.pub4.publish(emergen_call)
        rospy.logwarn('!!!emergency called ROS_control.py!!!')
        


    def drive_on(self):
        """drive_on"""
        msg = String()
        msg.data = "on"
        self.pub5.publish(msg) 
        return

    def drive_off(self):
        """drive_off"""
        msg = String()
        msg.data = "off"
        self.pub5.publish(msg)
        return

    def contactor_on(self):
        msg = String()
        msg.data = "on"
        self.pub6.publish(msg)
        return

    def contactor_off(self):
        msg = String()
        msg.data = "off"
        self.pub6.publish(msg)
        return

    def velocity_move(self, az_speed, el_speed, dist_arcsec = 5 * 3600, limit=True):
        vel = Velocity_mode_msg()
        vel.az_speed = az_speed
        vel.el_speed = el_speed
        vel.dist = dist_arcsec
        vel.limit = limit
        self.pub7.publish(vel)
        rospy.loginfo(vel)
        return

    def azel_move(self,az, el, off_x = 0, off_y = 0, offcoord = 'HORIZONTAL', hosei = 'hosei_230.txt',  lamda=2600, dcos=0, vel_x=0, vel_y=0,limit=True):
        """azel_move"""
        # az,el,off_x,off_y = deg
        # vel_x, vel_y = arcsec/s
        msg = Move_mode_msg()
        msg.x = az
        msg.y = el
        msg.code_mode = "horizontal"
        msg.off_x = off_x
        msg.off_y = off_y
        msg.hosei = hosei
        msg.offcoord = offcoord
        msg.lamda = lamda
        msg.dcos = dcos
        msg.vel_x = vel_x
        msg.vel_y = vel_y
        msg.limit = limit
        rospy.loginfo(msg)
        self.pub17.publish(msg)
        return
      

    def radec_move(self, ra, dec, code_mode, off_x = 0, off_y = 0, offcoord = 'HORIZONTAL', hosei = 'hosei_230.txt',  lamda=2600, dcos=0, az_rate=12000, el_rate=12000,limit=True):
        #self.ant.radec_move(ra, dec, code_mode, off_x, off_y, hosei, offcoord, lamda, az_rate, el_rate, dcos)
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
        msg.limit = limit
        #mv.az_rate ... no inplementation
        #mv.el_rate ... no inplementation
        rospy.loginfo(msg)
        self.pub8.publish(msg)
        return
    

    def galactic_move(self, l, b, off_x = 0, off_y = 0, offcoord = 'HORIZONTAL', hosei = 'hosei_230.txt', lamda=2600, az_rate=12000, el_rate=12000, dcos=0, limit=True):
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
        msg.limit = limit
        #mv.az_rate ... no inplementation
        #mv.el_rate ... no inplementation
        rospy.loginfo(msg)
        self.pub9.publish(msg)
        return

    def planet_move(self, number, off_x = 0, off_y = 0, offcoord = 'HORIZONTAL', hosei = 'hosei_230.txt', lamda=2600, az_rate=12000, el_rate=12000, dcos=0, limit=True):
        """1.Mercury 2.Venus 3. 4.Mars 5.Jupiter 6.Saturn 7.Uranus 8.Neptune, 9.Pluto, 10.Moon, 11.Sun"""
        msg = Move_mode_msg()
        msg.ntarg = number
        msg.off_x = off_x
        msg.off_y = off_y
        msg.code_mode = "planet"
        msg.hosei = hosei
        msg.offcoord = offcoord
        msg.lamda = lamda
        msg.dcos = dcos
        msg.limit = limit
        #mv.az_rate ... no inplementation
        #mv.el_rate ... no inplementation
        rospy.loginfo(msg)
        self.pub10.publish(msg)
        return

    def move_stop(self):
        msg = String()
        msg.data = "stop"
        print("move_stop")
        self.pub11.publish(msg)
        return

    def otf_scan(self, lambda_on, beta_on, coord_sys, dx, dy, dt, num, rampt, delay, lamda, hosei, code_mode, off_x, off_y, off_coord, dcos=0, ntarg = 0, limit=True):
        #on_start = self.ant.otf_start(lambda_on, beta_on, dcos, coord_sys, dx, dy, dt, num, rampt, delay, lamda, hosei, code_mode, off_x, off_y, off_coord, ntarg)
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
        msg.limit = limit
        rospy.loginfo(msg)
        self.pub12.publish(msg)
        
        return 

    def read_track(self):
        ret = self.ant.read_track()
        if ret[0] == "TRUE" and ret[1] == "TRUE":
            flag = True
        else:
            flag = False
        return flag


    def dome_move(self,dist):
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_move'
        self.pub13.publish(dome)
        dome = Dome_msg()
        dome.name = 'target_az'
        dome.value = str(dist)
        self.pub13.publish(dome)

    
    def dome_open(self):
        #"""Dome\u306eopen"""
        #self.ant.dome_open()
        #return
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_open'
        self.pub13.publish(dome)
    
    def dome_close(self):
        #"""Dome\u306eclose"""
        #self.ant.dome_close()
        #return
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_close'
        self.pub13.publish(dome)
        
    def memb_open(self):
        """\u30e1\u30f3\u30d6\u30ec\u30f3\u306eopen"""
        #self.ant.memb_open()
        #return
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'memb_open'
        self.pub13.publish(dome)

    def memb_close(self):
        #"""\u30e1\u30f3\u30d6\u30ec\u30f3\u306eopenclose"""
        #self.ant.memb_close()
        #return
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'memb_close'
        self.pub13.publish(dome)
        
    #def dome_move(self, dome_az):
       # """Dome\u3092(dome_az)\u306b\u52d5\u4f5c"""
        #self.ant.dome_move(dome_az)
        #return

    def dome_stop(self):
        """Dome\u306eclose\u52d5\u4f5c\u3092\u505c\u6b62"""
        #self.dome_track_end()
        #self.ant.dome_stop()
        #return
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_stop'
        self.pub13.publish(dome)
        
    def dome_track(self):
        """Dome\u3068\u671b\u9060\u93e1\u306esync"""
        #self.ant.dome_track()
        #return
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_tracking'
        self.pub13.publish(dome)

    def dome_track_end(self):
        """Dome\u3068\u671b\u9060\u93e1\u306esync\u306e\u7d42\u4e86"""
        #self.ant.dome_track_end()
        #return
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_track_end'
        self.pub13.publish(dome)

    def dome_tracking(self, req):
        self.dome_tracking_flag = req.data
        return

    def dome_tracking_check(self):
        rospy.loginfo(" dome_tracking now... \n")
        while not self.dome_tracking_flag:
            time.sleep(0.01)
            pass

# ===================
# mirror
# ===================

    def move_m4(self, position):
        """mirror4\u3092\u52d5\u304b\u3059("in"or"out")"""
        status = String()
        status.data = position
        self.pub14.publish(status)
        return

    
    def move_hot(self, position):
        """hotload\u3092\u52d5\u304b\u3059("in"or"out")"""
        #if position == "in": self.beam.hot_in()
        #elif position == "out": self.beam.hot_out()
        #else : print('set hotposition error')
        #return
        status = String()
        status.data = position
        self.pub15.publish(status)
        return

    def m2_move(self, dist):
        """m2\u3092\u52d5\u304b\u3059(um)"""
        status = Int64()
        status.data = dist
        self.pub16.publish(status)
        return

# ===================
# encoder
# ===================

    def observation(self, command, exposure):
        msg = Bool()
        if command == "start":
            msg.data = True
            #msg.data2 = exposure                                                                                                                                                 
        elif command == "end":
            msg.data = False
        else:
            rospy.logerr("argument is error!!")
            sys.exit()
        self.pub1.publish(msg)
        return

# ===================
# spectrometer
# ===================

    def oneshot_achilles(self, repeat=1, exposure=1.0, stime=0.0):
        # only python2
        sys.path.append("/home/amigos/ros/src/necst/lib")
        import achilles
        self.dfs = achilles.dfs()
        data = self.dfs.oneshot(repeat, exposure, stime)
        data_dict = {'dfs1': data[0], 'dfs2': data[1]}
        return data_dict

    def spectrometer(self, exposure):
        msg = Float64()
        msg.data = exposure
        self.pub3.publish(msg)
        return


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
  

