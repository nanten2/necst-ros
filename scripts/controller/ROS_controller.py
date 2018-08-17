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
import rosnode
from necst.msg import Move_mode_msg
from necst.msg import Otf_mode_msg
from necst.msg import Dome_msg
from necst.msg import Read_status_msg
from necst.msg import Achilles_msg
from necst.msg import Bool_necst
from necst.msg import String_necst
from necst.msg import Int64_necst
from necst.msg import Status_obs_msg
from necst.msg import Status_onepoint_msg
from nascorx_xffts.msg import XFFTS_para_msg
sys.path.append("/home/amigos/ros/src/necst/lib")
import achilles

class controller(object):

    task_flag = False
    antenna_tracking_flag = False
    dome_tracking_flag = False
    access_authority = "no_data"

    status = ""

    auth = ""
    frame = "controller"
    node_name = ""

    """otf parameter"""
    active = False
    obsmode = ""
    obs_script = ""
    obs_file = ""
    target = ""
    num_on = 0
    num_seq = 0
    xgrid = 0
    ygrid = 0
    exposure_hot = 0
    exposure_off = 0
    exposure_on = 0
    scan_direction = ""
    current_line = 0
    current_position = ""

    """ps parameter"""
    num_on = 0
    num_seq = 0
    
    def __init__(self, escape=False):
        """node registration
        *** warning!!! ***
        if you escape from authority_check, 
            escape -> node_name
        you can start flee node_name
        ******************
        """

        self.escape = escape            
        if self.escape:
            print("Authority escape mode start.")            
            self.node_name = self.escape
        else:
            self.node_name = self.initialize()
            pass
        rospy.init_node(self.node_name)
        
        """init"""
        self.antenna_sub = rospy.Subscriber("tracking_check", Bool_necst, self._antenna_tracking)
        self.dome_sub = rospy.Subscriber("dome_tracking_check", Bool_necst, self._dome_tracking)
        self.regist_sub = rospy.Subscriber("authority_check", String_necst, self._pick_up, queue_size=1)
        self.obs_stop_sub = rospy.Subscriber("obs_stop", String_necst, self._obs_stop, queue_size=1)
        self.pub_obs_stop = rospy.Publisher("obs_stop", String_necst, queue_size=1)        
        self.pub_drive = rospy.Publisher("antenna_drive", String_necst, queue_size = 1)
        self.pub_contactor = rospy.Publisher("antenna_contactor", String_necst, queue_size = 1)
        self.pub_onepoint = rospy.Publisher("onepoint_command", Move_mode_msg, queue_size=1, latch=True)
        self.pub_linear = rospy.Publisher("linear_command", Move_mode_msg, queue_size=1, latch=True)        
        self.pub_planet = rospy.Publisher("planet_command", Move_mode_msg, queue_size=1, latch=True)        
        self.pub_stop = rospy.Publisher("move_stop", Bool_necst, queue_size = 1, latch = True)
        self.pub_otf = rospy.Publisher("antenna_otf", Otf_mode_msg, queue_size = 1, latch = True)
        self.pub_planet_scan = rospy.Publisher("planet_otf", Otf_mode_msg, queue_size = 1, latch = True)        
        self.pub_dome = rospy.Publisher("dome_move", Dome_msg, queue_size = 1, latch = True)
        self.pub_dome_move = rospy.Publisher("dome_move_az", Dome_msg, queue_size = 1, latch = True)
        self.pub_m4 = rospy.Publisher('m4', String_necst, queue_size = 1, latch = True)
        self.pub_hot = rospy.Publisher("hot", String_necst, queue_size = 1, latch = True)
        self.pub_m2 = rospy.Publisher("m2", Int64_necst, queue_size=1, latch=True)
        self.pub_achilles = rospy.Publisher("achilles", Achilles_msg, queue_size=1)
        self.pub_XFFTS = rospy.Publisher("XFFTS_parameter", XFFTS_para_msg, queue_size=1)
        self.pub_regist = rospy.Publisher("authority_regist", String_necst, queue_size=1)
        self.pub_obsstatus = rospy.Publisher("obs_status", Status_obs_msg, queue_size=1)
        self.pub_onestatus = rospy.Publisher("one_status", Status_onepoint_msg, queue_size=1)        
        self.pub_queue = rospy.Publisher("queue_obs", Bool_necst, queue_size=1)
        self.pub_alert = rospy.Publisher("alert", String_necst, queue_size=1)
        time.sleep(0.5)# authority regist time                

        """get authority"""
        if not self.escape:
            self.get_authority()
        
        """finish action"""
        atexit.register(self._release)

        time.sleep(1.)# authority regist time        
        
        return
    
# ===================
# finish action
# ===================
    def _release(self):
        self.antenna_sub.unregister()
        self.dome_sub.unregister()
        #self.read_sub.unregister()
        self.regist_sub.unregister()
        print("ROS_controller is finished.")
        return

# ===================
# authority
# ===================

    
    def deco_check(func):
        import functools
        @functools.wraps(func)
        def wrapper(self, *args,**kwargs):
            #self.get_authority()
            time.sleep(0.5)
            if self.escape:
                ret = func(self, *args,**kwargs)
            elif self.auth == self.node_name:
                ret = func(self, *args,**kwargs)
            else:
                ret = ""
                print("This node don't have authority...")
                print("current authority : ", self.auth)
                pass
            return ret
        return wrapper
    
    def _pick_up(self,req):
        self.auth = req.data
        return
    
    def initialize(self):
        if self.node_name:
            print("You already have node_name : ", self.node_name)
            return
        else:
            pass
        for i in range(100):
            name = self.frame +str(i)
            node_data = rosnode.get_node_names()
            if ("/" + name) in node_data:
                pass
            else:
                break
        self.node_name = name
        print("node_name is ", self.node_name)
        return name

    def registration(self, name, user=""):
        msg = String_necst()
        if user == "master":
            msg.data = name
            msg.from_node = "master"
            msg.timestamp = time.time()
        else:
            msg.data = name
            msg.from_node = self.node_name
            msg.timestamp = time.time()
        self.pub_regist.publish(msg)
        return
    
    def get_authority(self):
        self.registration(self.node_name)
        return

    def release_authority(self):
        self.registration("")
        return

    def check_my_node(self):
        return self.node_name

# ===================
# antenna
# ===================
    
    @deco_check
    def drive(self, switch = ""):
        """change drive

        Parameters
        ----------
        swith : on or off
        
        """
        msg = String_necst()
        self.move_stop()
        if not switch:
            switch = str(input("drive change (on/off) : ")).lower()
        msg.data =  switch
        msg.from_node = self.node_name
        msg.timestamp = time.time()
        if switch in ["on", "off", "ON", "OFF"]:
            self.pub_drive.publish(msg)
            self.pub_contactor.publish(msg)
            print("drive : ", switch, "!!")
        else:
            print("!!bad command!!")
            pass
        return

    @deco_check
    def onepoint_move(self, x, y, coord="altaz", off_x=0, off_y=0, offcoord='altaz', hosei='hosei_230.txt',  lamda=2600, dcos=0, limit=True,):
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
        limit    : soft limit [az:-240~240, el:30~80] (True:limit_on, False:limit_off)
        """
        self.pub_onepoint.publish(x, y, coord, "", off_x, off_y, offcoord, hosei, lamda, dcos, limit, self.node_name, time.time())
        return
    
    @deco_check
    def planet_move(self, planet, off_x=0, off_y=0, offcoord="altaz", hosei="hosei_230.txt", lamda=2600, dcos=0, limit=True):
        """ planet_move
        
        Parameters
        ----------
        planet   : planet_number or name 
                   1.Mercury 2.Venus 4.Mars 5.Jupiter 6.Saturn 7.Uranus 8.Neptune, 9.Pluto, 10.Moon, 11.Sun
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
        print("planet name is ", planet)
        self.pub_planet.publish(0, 0, "planet", planet, off_x, off_y, offcoord, hosei, lamda, dcos,limit, self.node_name, time.time())
        return
        
    @deco_check
    def linear_move(self, x, y, coord="altaz", dx=0, dy=0, offcoord='altaz', hosei='hosei_230.txt',  lamda=2600, dcos=0, limit=True,):
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
        limit    : soft limit [az:-240~240, el:30~80] (True:limit_on, False:limit_off)
        """

        self.pub_linear.publish(x, y, coord, "", dx, dy, offcoord, hosei, lamda, dcos, limit, self.node_name, time.time())
        return
    
    @deco_check
    def otf_scan(self, x, y, coord, dx, dy, dt, num, rampt, delay, current_time,  off_x=0, off_y=0, offcoord="j2000", dcos=0, hosei="hosei_230.txt", lamda=2600., limit=True):
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
        current_time    : time.time()
        off_x    : (target_x)-(scan start_x) [arcsec]
        off_y    : (target_y)-(scan start_y) [arcsec]
        offcoord : equal coord (no implementation)
        dcos     : projection (no:0, yes:1)
        hosei    : hosei file name (default ; hosei_230.txt)
        lamda    : observation wavelength [um] (default ; 2600)
        limit    : soft limit [az:-240~240, el:30~80] (True:limit_on, False:limit_off)
        """
        print("start OTF scan!!")

        self.pub_otf.publish(x, y, coord, dx, dy, dt, num, rampt,
                             delay, off_x, off_y, offcoord,
                             dcos, hosei, lamda, limit, self.node_name,
                             current_time)
        
        return
    
    @deco_check
    def planet_scan(self, planet, dx, dy, dt, num, rampt, delay, current_time,  off_x=0, off_y=0, offcoord="j2000", dcos=0, hosei="hosei_230.txt", lamda=2600., limit=True):
        """ planet otf scan

        Parameters
        ----------
        planet   : planet name
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
        self.pub_planet_scan.publish(0, 0, planet, dx, dy, dt, num, rampt,
                             delay, off_x, off_y, offcoord,
                             dcos, hosei, lamda, limit, self.node_name,
                             current_time)        
        return
    

    def queue_observation(self, flag):
        """ queue observation
        Parameters
        ----------        
        flag : start or stop
        """
        if flag == "start":
            self.registration("", "master")
            self.pub_queue.publish(True, self.node_name, time.time())
        elif flag == "stop":
            self.pub_queue.publish(False, self.node_name, time.time())            
        else:
            print("Bad command...")
            print("Please 'start' or 'stop'.")
        return
        

    def _antenna_tracking(self, req):
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

    def _obs_stop(self, req):
        self.registration("", "master")
        self.move_stop()
        print(req.data)
        return
    
    def move_stop(self):
        print("move_stop")
        self.pub_stop.publish(True, self.node_name, time.time())
        time.sleep(0.2)
        return

# ===================
# dome
# ===================
    
    @deco_check    
    def dome(self, value):
        """dome controll

        Parameter
        =========
        value : "open"         --> dome_open()
                "close"        --> dome_close()
                distance [deg] --> dome_move(value)
                "stop"         --> dome_stop()
        """
        if isinstance(value,int) or isinstance(value, float):
            self.dome_move(value)
        elif value.lower() == "open":
            self.dome_open()
        elif value.lower() == "close":
            self.dome_close()
        elif value.lower() == "stop":
            self.dome_stop()
        else:
            print("parameter error")
            pass
        return

    @deco_check    
    def dome_move(self,dist):
        """ dome move
        
        Parameters
        ----------
        dist : distance [deg]
        """
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_move'
        dome.from_node = self.node_name
        dome.timestamp = time.time()
        self.pub_dome.publish(dome)
        dome_move = Dome_msg()
        dome_move.name = 'target_az'
        dome_move.value = str(dist)
        dome_move.from_node = self.node_name
        dome_move.timestamp = time.time()
        self.pub_dome_move.publish(dome_move)
        return

    @deco_check    
    def dome_open(self):
        """Dome open"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_open'
        dome.from_node = self.node_name
        dome.timestamp = time.time()        
        self.pub_dome.publish(dome)
        return
    
    def dome_close(self):
        """Dome close"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_close'
        dome.from_node = self.node_name
        dome.timestamp = time.time()        
        self.pub_dome.publish(dome)
        return

    @deco_check    
    def memb(self, value):
        """memb move

        Parameter
        =========
        value : "open"  --> memb_open()
                "close" --> memb_close()
        """
        if value.lower() == "open":
            self.memb_open()
        elif value.lower() == "close":
            self.memb_close()
        else:
            print("parameter error")
            pass
        return

    @deco_check        
    def memb_open(self):
        """membrane open"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'memb_open'
        dome.from_node = self.node_name
        dome.timestamp = time.time()        
        self.pub_dome.publish(dome)
        return

    def memb_close(self):
        """membrane close"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'memb_close'
        dome.from_node = self.node_name
        dome.timestamp = time.time()        
        self.pub_dome.publish(dome)
        return

    def dome_stop(self):
        """Dome stop"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_stop'
        dome.from_node = self.node_name
        dome.timestamp = time.time()
        self.pub_dome.publish(dome)
        return

    @deco_check    
    def dome_track(self):
        """Dome sync antenna_az"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_tracking'
        dome.from_node = self.node_name
        dome.timestamp = time.time()        
        self.pub_dome.publish(dome)
        return

    def dome_track_end(self):
        """Dome stop antenna_az sync"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_track_end'
        dome.from_node = self.node_name
        dome.timestamp = time.time()        
        self.pub_dome.publish(dome)
        return

    def _dome_tracking(self, req):
        self.dome_tracking_flag = req.data
        return

    @deco_check    
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
    @deco_check
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

    @deco_check    
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
    
    @deco_check
    def move_m2(self, dist):
        """m2 move

        Parameter
        ---------
        dist : distance [um]
        
        """
        status = Int64_necst()
        status.data = int(dist)
        status.from_node = self.node_name
        status.timestamp = time.time()        
        self.pub_m2.publish(status)
        return

# ===================
# encoder
# ===================

    @deco_check
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
    @deco_check
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

    @deco_check
    def old_oneshot(self, repeat=1, exposure=1.0, stime=0.0):
        dfs = achilles.dfs()
        data = dfs.oneshot(req.repeat, req.exposure, req.stime)
        data_dict = {'dfs1': data[0], 'dfs2': data[1]}
        return data_dict
    
    @deco_check    
    def spectrometer(self, exposure):
        msg = Float64()
        msg.data = exposure
        self.pub3.publish(msg)
        return
    
    def oneshot_XFFTS(self, integtime, repeat, synctime):
        """XFFTS Publisher
        Parameters
        ----------
        integtime : integration time of each observation.
        repeat    : How many times to repeat observation.
        """
        
        msg = XFFTS_para_msg()
        msg.integtime = integtime
        msg.repeat = repeat
        msg.synctime = synctime
        msg.from_node = self.node_name
        msg.timestamp = time.time()
        
        self.pub_XFFTS.publish(msg)
        return
        

    # ===================
    # status
    # ===================

    def onepoint_status(self, active=False, target="", num_on=0, num_seq=0, exposure_hot=0, exposure_off=0, exposure_on=0, current_num=0, current_position=""):
        """observation status
        this function is used by ROS_onepoint.py etc...
        =======================================================
        using parameter at observation_start is  1 ~ 9
        using parameter at getting_data_time is 1 and 8, 9        
        using parameter at observation_end is 1
        ========================================================

        Parameter
        ---------
        1.active : (start --> True) or (end --> False)
        2.target : object name
        3.num_on : number of on_position between off_position(p/s --> 1,  radio_pointing --> 3or5)
        4.num_seq: number of observation sequence
        5.exposure_hot : exposure time[s] at hot
        6.exposure_off : exposure time[s] at off
        6.exposure_on : exposure time[s] at on

        8.current_num : current number (1~len(num_seq))
        9.current_position : position (hot or off or on)

        """
        if target != "":
            self.active = active
            self.target = target
            self.num_on = num_on
            self.num_seq = num_seq
            self.exposure_hot = exposure_hot
            self.exposure_off = exposure_off
            self.exposure_on = exposure_on
            
        msg = Status_onepoint_msg()
        msg.active = active
        msg.target = self.target
        msg.num_on = self.num_on
        msg.num_seq = self.num_seq
        msg.exposure_hot = self.exposure_hot
        msg.exposure_off = self.exposure_off
        msg.exposure_on = self.exposure_on
        msg.current_num = current_num
        msg.current_position = current_position
        msg.from_node = self.node_name
        msg.timestamp = time.time()
        
        self.pub_onestatus.publish(msg)
        return

    def obs_status(self, active=False, obsmode="", obs_script="", obs_file="", target="", num_on=0, num_seq=0, xgrid=0, ygrid=0, exposure_hot=0, exposure_off=0, exposure_on=0, scan_direction="", current_num=0, current_position=""):
        """observation status
        this function is used by obs_script
        =======================================================
        using parameter at observation_start is  1 ~ 13
        using parameter at getting_data_time is 1 and 14, 15        
        using parameter at observation_end is 1
        ========================================================
        ver. 2018/07/26

        Parameter
        ---------
        1.active         : (start --> True) or (end --> False)
        2.obsmode        : otf or ps or radio or etc...
        3.obs_script     : obsscript name
        4.obs_file       : obsfile name
        5.target         : object name
        6.num_on         : number of on_position between off_position
        7.num_seq        : number of scan
        8.xgrid          : length xgrid [arcsec]
        9.ygrid          : length xgrid [arcsec]
        10.exposure_hot   : exposure time[s] at hot
        11.exposure_off   : exposure time[s] at off
        12.exposure_on   : exposure time[s] at on
        13.scan_direction: scan direction(x or y)

        14.current_num      : number of current point (otf : 0 ~ num_on*num_seq)
        15.current_position : position (hot or off or on)

        """
        if target != "":
            self.active = active
            self.obsmode = obsmode
            self.obs_script = obs_script
            self.obs_file = obs_file
            self.target = target
            self.num_on = num_on
            self.num_seq = num_seq
            self.xgrid = xgrid
            self.ygrid = ygrid
            self.exposure_hot = exposure_hot
            self.exposure_off = exposure_off
            self.exposure_on = exposure_on
            self.scan_direction = scan_direction
            
        msg = Status_obs_msg()
        msg.active = active
        msg.obsmode = self.obsmode
        msg.obs_script = self.obs_script
        msg.obs_file = self.obs_file
        msg.target = self.target        
        msg.num_on = self.num_on
        msg.num_seq = self.num_seq
        msg.xgrid = self.xgrid
        msg.ygrid = self.ygrid
        msg.exposure_hot = self.exposure_hot
        msg.exposure_off = self.exposure_off
        msg.exposure_on = self.exposure_on
        msg.scan_direction = self.scan_direction
        msg.current_num = current_num
        msg.current_position = current_position
        msg.from_node = self.node_name
        msg.timestamp = time.time()
        
        self.pub_obsstatus.publish(msg)
        return

    
    @deco_check
    def read_status(self):
        """read status

        attention!!
        -----------
        you need execution this script.

        $rosrun necst ROS_status.py 1
        or 
        $python ROS_status.py 1

        """
        self.read_sub = rospy.Subscriber("read_status", Read_status_msg, self._write_status)
        
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
    
    def _write_status(self, req):
        self.status = req
        self.read_sub.unregister()
        return

    # ===================
    # others
    # ===================

    
    def alert(self, message, emergency=False):
        msg = String_necst()
        msg.data = message
        msg.from_node = self.node_name
        msg.timestamp = time.time()
        self.pub_alert.publish(msg)
        if emergency:
            self.pub_obs_stop.publish(message, self.node_name, time.time())
        else:
            pass
        return


    
