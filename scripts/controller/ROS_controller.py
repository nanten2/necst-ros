#! /usr/bin/env python3
# coding:utf-8

"""
------------------------------------------------
[History]
2017/10/18 : kondo takashi
2018/05/07 : kondo
------------------------------------------------
"""
import os
import sys
import time
import atexit
from datetime import datetime as dt
import rospy
from necst.msg import Move_mode_msg
from necst.msg import Otf_mode_msg
from necst.msg import Dome_msg
from necst.msg import Read_status_msg
from necst.msg import Achilles_msg
from necst.msg import Bool_necst
from necst.msg import String_necst
from necst.msg import Int64_necst
sys.path.append("/home/amigos/ros/src/necst/lib")
import node_authority
auth = node_authority.authority()

        
class controller(object):

    task_flag = False
    antenna_tracking_flag = False
    dome_tracking_flag = False
    access_authority = "no_data"

    status = ""
    

    def __init__(self):
        """start authority check"""
        self.node_name = auth.initialize()
        aa = self.node_name
        rospy.init_node(aa)
        auth.start()
        
        """init"""
        self.read_sub = rospy.Subscriber("read_status", Read_status_msg, self.write_status)        
        self.antenna_sub = rospy.Subscriber("tracking_check", Bool_necst, self.antenna_tracking)
        self.dome_sub = rospy.Subscriber("dome_tracking_check", Bool_necst, self.dome_tracking)

        self.pub_drive = rospy.Publisher("antenna_drive", String_necst, queue_size = 1)
        self.pub_contactor = rospy.Publisher("antenna_contactor", String_necst, queue_size = 1)
        self.pub_onepoint = rospy.Publisher("onepoint_command", Move_mode_msg, queue_size=1, latch=True)
        self.pub_linear = rospy.Publisher("linear_command", Move_mode_msg, queue_size=1, latch=True)        
        self.pub_planet = rospy.Publisher("planet_command", Move_mode_msg, queue_size=1, latch=True)        
        self.pub_stop = rospy.Publisher("move_stop", Bool_necst, queue_size = 1, latch = True)
        self.pub_otf = rospy.Publisher("antenna_otf", Otf_mode_msg, queue_size = 1, latch = True)
        self.pub_planet_scan = rospy.Publisher("planet_otf", Otf_mode_msg, queue_size = 1, latch = True)        
        self.pub_dome = rospy.Publisher("dome_move", Dome_msg, queue_size = 1, latch = True)
        self.pub_m4 = rospy.Publisher('m4', String_necst, queue_size = 1, latch = True)
        self.pub_hot = rospy.Publisher("hot", String_necst, queue_size = 1, latch = True)
        self.pub_m2 = rospy.Publisher("m2", Int64_necst, queue_size=1, latch=True)
        self.pub_achilles = rospy.Publisher("achilles", Achilles_msg, queue_size=1)
        time.sleep(0.5)
        atexit.register(self.release)
        return

    def release(self):
        self.antenna_sub.unregister()
        self.dome_sub.unregister()
        self.read_sub.unregister()
        print("ROS_controller is finished.")
        return
    
    def get_authority(self):
        auth.registration(self.node_name)
        return

    def release_authority(self):
        auth.registration("")

    def antenna_tracking(self, req):
        self.antenna_tracking_flag = req.data
        return

    def antenna_tracking_check(self):
        """antenna_tracking_check"""
        rospy.loginfo(" tracking now... \n")
        time.sleep(3.)
        while not self.antenna_tracking_flag:
            time.sleep(0.01)
            pass
        return
    
    @auth.deco_check
    def drive(self, switch = ""):
        """change drive

        Parameters
        ----------
        swith : on or off
        
        """
        msg = String_necst()
        self.move_stop()
        if not switch:
            switch = str(input("drive change (on/off) : "))
        msg.data =  switch.lower()
        msg.from_node = self.node_name
        msg.timestamp = time.time()
        if switch == "on" or switch == "off":
            self.pub_drive.publish(msg)
            self.pub_contactor.publish(msg)
            print("drive : ", switch, "!!")
        else:
            print("!!bad command!!")
            pass
        return

    @auth.deco_check
    def onepoint_move(self, x, y, coord="altaz", off_x=0, off_y=0, offcoord='altaz', hosei='hosei_230.txt',  lamda=2600, dcos=0, func_x="", func_y="", movetime=10, limit=True,):
        """ azel_move, radec_move, galactic_move
        
        Parameters
        ----------
        x        : target_x [deg]
        y        : target_y [deg]
        coord    : "altaz" or "j2000" or "b1950" or "galactic"  
        off_x    : offset_x [arcsec]
        off_y    : offset_y [arcsec]
        offcoord : "altaz" or "j2000" or "b1950" or "galactic" 
        hosei    : hosei file name (default ; hosei_230.txt)
        lamda    : observation wavelength [um] (default ; 2600)
        dcos     : projection (no:0, yes:1)
        func_x   : free scan [arcsec/s] (cf:20*x or math.sin(x) or etc...)
        func_y   : free scan [arcsec/s] (cf:20*y or math.sin(y) or etc...)
        movetime : azel_list length [s]
        limit    : soft limit [az:-240~240, el:30~80] (True:limit_on, False:limit_off)
        assist   : ROS_antenna_assist is on or off (True:on, False:off)
        """
        #self.pub_stop.publish(False, self.node_name, time.time())
        self.pub_onepoint.publish(x, y, coord, "", off_x, off_y, offcoord, hosei, lamda, dcos, str(func_x), str(func_y), limit, self.node_name, time.time())
        return
    
    @auth.deco_check
    def planet_move(self, planet, off_x=0, off_y=0, offcoord="altaz", hosei="hosei_230.txt", lamda=2600, dcos=0, limit=True):
        """ planet_move
        
        Parameters
        ----------
        planet   : planet_number (only when using "planet_move"!!)
                   1.Mercury 2.Venus 3. 4.Mars 5.Jupiter 6.Saturn 7.Uranus 8.Neptune, 9.Pluto, 10.Moon, 11.Sun
        off_x    : offset_x [arcsec]
        off_y    : offset_y [arcsec]
        offcoord : "altaz" or "j2000" or "b1950" or "galactic" 
        hosei    : hosei file name (default ; hosei_230.txt)
        lamda    : observation wavelength [um] (default ; 2600)
        dcos     : projection (no:0, yes:1)
        limit    : soft limit [az:-240~240, el:30~80] (True:limit_on, False:limit_off)
        """
        if isinstance(planet, int):
            planet_list = {1:"mercury", 2:"venus", 4:"mars", 5:"jupiter",6:"saturn", 7:"uranus", 8:"neptune", 10:"moon", 11:"sun"}
            planet = planet_list[int(planet)]
        else:
            pass
        #self.pub_stop.publish(False, self.node_name, time.time())
        print("planet name is ", planet)
        self.pub_planet.publish(0, 0, "planet", planet, off_x, off_y, offcoord, hosei, lamda, dcos, "0","0",limit, self.node_name, time.time())
        return
        
    @auth.deco_check
    def linear_move(self, x, y, coord="altaz", dx=0, dy=0, offcoord='altaz', hosei='hosei_230.txt',  lamda=2600, dcos=0, func_x="", func_y="", limit=True,):
        """ azel_move, radec_move, galactic_move
        
        Parameters
        ----------
        x        : target_x [deg]
        y        : target_y [deg]
        coord    : "altaz" or "j2000" or "b1950" or "galactic"  
        dx    : delta_x [arcsec/s]
        dy    : delta_y [arcsec/s]
        offcoord : "altaz" or "j2000" or "b1950" or "galactic" 
        hosei    : hosei file name (default ; hosei_230.txt)
        lamda    : observation wavelength [um] (default ; 2600)
        dcos     : projection (no:0, yes:1)
        func_x   : free scan [arcsec/s] (cf:20*x or math.sin(x) or etc...)
        func_y   : free scan [arcsec/s] (cf:20*y or math.sin(y) or etc...)
        limit    : soft limit [az:-240~240, el:30~80] (True:limit_on, False:limit_off)
        """
        #self.pub_stop.publish(False, self.node_name, time.time())
        self.pub_linear.publish(x, y, coord, "", dx, dy, offcoord, hosei, lamda, dcos, str(func_x), str(func_y), limit, self.node_name, time.time())
        return
    
    @auth.deco_check
    def otf_scan(self, x, y, coord, dx, dy, dt, num, rampt, delay, start_on,  off_x=0, off_y=0, offcoord="j2000", dcos=0, hosei="hosei_230.txt", lamda=2600., limit=True):
        """ otf scan

        Parameters
        ----------
        x        : target_x [deg]
        y        : target_y [deg]
        coord    : "j2000" or "b1950" or "galactic"
        dx       : x_grid length [arcsec]
        dy       : y_grid length [arcsec]
        dt       : exposure time [s]
        num      : scan point [ num / 1 line]
        rampt    : ramp time [s]
        delay    : (start observation time)-(now time) [s]
        start_on : start on position scan [utctime]
        off_x    : (target_x)-(scan start_x) [arcsec]
        off_y    : (target_y)-(scan start_y) [arcsec]
        offcoord : equal coord (no implementation)
        dcos     : projection (no:0, yes:1)
        hosei    : hosei file name (default ; hosei_230.txt)
        lamda    : observation wavelength [um] (default ; 2600)
        movetime : azel_list length [s] (otf_mode = 0.01)
        limit    : soft limit [az:-240~240, el:30~80] (True:limit_on, False:limit_off)
        """
        current_time = time.time()
        print("start OTF scan!!")
        #self.pub_stop.publish(False, self.node_name, time.time())
        self.pub_otf.publish(x, y, coord, dx, dy, dt, num, rampt,
                             delay, start_on, off_x, off_y, offcoord,
                             dcos, hosei, lamda, limit, self.node_name,
                             current_time)
        
        return
    
    @auth.deco_check
    def planet_scan(self):
        """ planet otf scan

        Parameters
        ----------
        x        : target_x [deg]
        y        : target_y [deg]
        coord    : "j2000" or "b1950" or "galactic"
        dx       : x_grid length [arcsec]
        dy       : y_grid length [arcsec]
        dt       : exposure time [s]
        num      : scan point [ num / 1 line]
        rampt    : ramp time [s]
        delay    : (start observation time)-(now time) [s]
        start_on : start on position scan [utctime]
        off_x    : (target_x)-(scan start_x) [arcsec]
        off_y    : (target_y)-(scan start_y) [arcsec]
        offcoord : equal coord (no implementation)
        dcos     : projection (no:0, yes:1)
        hosei    : hosei file name (default ; hosei_230.txt)
        lamda    : observation wavelength [um] (default ; 2600)
        movetime : azel_list length [s] (otf_mode = 0.01)
        limit    : soft limit [az:-240~240, el:30~80] (True:limit_on, False:limit_off)
        """
        
        print("start OTF scan!!")
        #self.pub_stop.publish(False,self.node_name, time.time())
        self.pub_planet_scan.publish(x, y, coord, dx, dy, dt, num,rampt,
                                      delay, start_on, off_x, off_y, offcoord,
                                      dcos, hosei, lamda, limit,
                                      self.node_name, time.time())
        return
        
    @auth.deco_check
    def move_stop(self):
        print("move_stop")
        self.pub_stop.publish(True, self.node_name, time.time())
        time.sleep(0.2)
        return

    @auth.deco_check    
    def dome(self, value):
        """dome controll

        Parameter
        =========
        value : "open"         --> dome_open()
                "close"        --> dome_close()
                distance [deg] --> dome_move(value)
                "stop"         --> dome_stop()
        """
        if value == "open":
            self.dome_open()
        elif value == "close":
            self.dome_close()
        elif value == "stop":
            self.dome_stop()
        elif isinstance(value,int) or isinstance(value, float):
            self.dome_move(dist)
        else:
            print("parameter error")
            pass
        return

    @auth.deco_check    
    def dome_move(self,dist):
        """ dome move
        
        Parameters
        ----------
        dist : distance [deg]
        """
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_move'
        self.pub_dome.publish(dome)
        dome = Dome_msg()
        dome.name = 'target_az'
        dome.value = str(dist)
        dome.from_node = self.node_name
        dome.timestamp = time.time()
        self.pub_dome.publish(dome)
        return

    @auth.deco_check    
    def dome_open(self):
        """Dome open"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_open'
        dome.from_node = self.node_name
        dome.timestamp = time.time()        
        self.pub_dome.publish(dome)
        return
    
    @auth.deco_check    
    def dome_close(self):
        """Dome close"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_close'
        dome.from_node = self.node_name
        dome.timestamp = time.time()        
        self.pub_dome.publish(dome)
        return

    @auth.deco_check    
    def memb(self, value):
        """memb move

        Parameter
        =========
        value : "open"  --> memb_open()
                "close" --> memb_close()
        """
        if value == "open":
            self.memb_open()
        elif value == "close":
            self.memb_close()
        else:
            print("parameter error")
            pass
        return

    @auth.deco_check        
    def memb_open(self):
        """membrane open"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'memb_open'
        dome.from_node = self.node_name
        dome.timestamp = time.time()        
        self.pub_dome.publish(dome)
        return

    @auth.deco_check    
    def memb_close(self):
        """membrane close"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'memb_close'
        dome.from_node = self.node_name
        dome.timestamp = time.time()        
        self.pub_dome.publish(dome)
        return

    @auth.deco_check    
    def dome_stop(self):
        """Dome stop"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_stop'
        dome.from_node = self.node_name
        dome.timestamp = time.time()
        self.pub_dome.publish(dome)
        return

    @auth.deco_check    
    def dome_track(self):
        """Dome sync antenna_az"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_tracking'
        dome.from_node = self.node_name
        dome.timestamp = time.time()        
        self.pub_dome.publish(dome)
        return

    @auth.deco_check
    def dome_track_end(self):
        """Dome stop antenna_az sync"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_track_end'
        dome.from_node = self.node_name
        dome.timestamp = time.time()        
        self.pub_dome.publish(dome)
        return

    def dome_tracking(self, req):
        self.dome_tracking_flag = req.data
        return

    @auth.deco_check    
    def dome_tracking_check(self):
        """dome tracking check"""
        rospy.loginfo(" dome_tracking now... \n")
        time.sleep(3.)
        while not self.dome_tracking_flag:
            time.sleep(0.01)
            pass

# ===================
# mirror
# ===================
    @auth.deco_check
    def move_m4(self, position):
        """mirror4 move
        
        Parameter
        ---------
        position : in or out
        
        """
        status = String_necst()
        status.data = position
        status.from_node = self.node_name
        status.timestamp = time.time()        
        self.pub_m4.publish(status)
        return

    @auth.deco_check    
    def move_hot(self, position):
        """hotload move

        Parameter
        ---------
        position : in or out
        """
        status = String_necst()
        status.data = position
        status.from_node = self.node_name
        status.timestamp = time.time()        
        self.pub_hot.publish(status)
        return
    
    @auth.deco_check
    def m2_move(self, dist):
        """m2 move

        Parameter
        ---------
        dist : distance [um]
        
        """
        status = Int64_necst()
        status.data = dist
        status.from_node = self.node_name
        status.timestamp = time.time()        
        self.pub_m2.publish(status)
        return

# ===================
# encoder
# ===================

    @auth.deco_check
    def observation(self, command, exposure):
        msg = Bool_necst()
        if command == "start":
            msg.data = True
            #msg.data2 = exposure                                                                                                                                        
        elif command == "end":
            msg.data = False
        else:
            rospy.logerr("argument is error!!")
            sys.exit()
        msg.from_node = self.node_name
        msg.timestamp = time.time()            
        self.pub1.publish(msg)
        return

# ===================
# spectrometer
# ===================
    @auth.deco_check
    def oneshot_achilles(self, repeat=1, exposure=1.0, stime=0.0):
        """get spectrum by ac240

        Parameter
        ---------
        repeat : repeat number
        exposure : exposure time [s]
        stime : start mjd time [day]
        
        """
        msg = Achilles_msg()
        msg.repeat = repeat
        msg.exposure = exposure
        msg.stime = stime
        msg.day = dt.utcnow().strftime("%y%m%d%H%M%S")
        msg.from_node = self.node_name
        msg.timestamp = time.time()        
        self.pub_achilles.publish(msg)
        dir_name = "/home/amigos/data/experiment/achilles/" + str(msg.day) + "/"
        file_name = msg.day + "_fin.txt"
        while not rospy.is_shutdown():
            if not os.path.exists(dir_name + file_name):
                print("get data now...")
                pass
            else:
                break
            time.sleep(1)
        f1 = open(dir_name+str(msg.day) + "_1.txt", "r")
        dfs1 = f1.readline()
        f1.close()
        f2 = open(dir_name+str(msg.day) + "_2.txt", "r")
        dfs2 = f2.readline()
        f2.close()
        data_dict = {'dfs1': eval(dfs1), 'dfs2': eval(dfs2)}
        return data_dict

    @auth.deco_check    
    def spectrometer(self, exposure):
        msg = Float64()
        msg.data = exposure
        self.pub3.publish(msg)
        return

# ===================
# status
# ===================
    @auth.deco_check
    def read_status(self):
        """read status

        attention!!
        -----------
        you need execution this script.

        $rosrun necst ROS_status.py 1
        or 
        $python ROS_status.py 1

        """
        while not rospy.is_shutdown():
            if self.status:
                status = self.status
                self.status = ""
                rospy.loginfo("read end")
                break
            else:
                rospy.loginfo("read now...")
                status = ""
                time.sleep(1.)

        return status
    
    def write_status(self, req):
        self.status = req
        self.read_sub.unregister()
        return




