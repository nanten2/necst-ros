#! /usr/bin/env python3
# coding:utf-8

import os
import sys
import time
import atexit
import functools
from datetime import datetime as dt
import rospy
import rosnode
import logger
from datetime import datetime
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
from necst.msg import oneshot_msg
#from nascorx_xffts.msg import XFFTS_para_msg
from necst.msg import xffts_flag_msg
from necst.msg import encdb_flag_msg
from necst.msg import textfile_msg
from necst.msg import analy_msg
from necst.msg import Center_beam_num_msg
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/amigos/ros/src/nasco_system/scripts")
import achilles
import nasco_controller
from necst.srv import ac240_srv
from necst.srv import ac240_srvResponse
from necst.srv import Bool_srv
from necst.srv import Bool_srvResponse
from std_msgs.msg import String

class controller(object):

    task_flag = False
    antenna_tracking_flag = False
    antenna_tracking_time = ""
    command_time = ""
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
        self.pub_onepoint = rospy.Publisher("onepoint_command", Move_mode_msg, queue_size=1)
        self.pub_linear = rospy.Publisher("linear_command", Move_mode_msg, queue_size=1)        
        self.pub_planet = rospy.Publisher("planet_command", Move_mode_msg, queue_size=1)        
        self.pub_stop = rospy.Publisher("move_stop", Bool_necst, queue_size = 1)
        self.pub_otf = rospy.Publisher("antenna_otf", Otf_mode_msg, queue_size = 1)
        self.pub_planet_scan = rospy.Publisher("planet_otf", Otf_mode_msg, queue_size = 1)        
        self.pub_dome = rospy.Publisher("dome_move", Dome_msg, queue_size = 1)
        self.pub_dome_move = rospy.Publisher("dome_move_az", Dome_msg, queue_size = 1)
        self.pub_m4 = rospy.Publisher('m4', String_necst, queue_size = 1)
        self.pub_hot = rospy.Publisher("hot", String_necst, queue_size = 1)
        self.pub_m2 = rospy.Publisher("m2", Int64_necst, queue_size=1)
        self.pub_achilles = rospy.Publisher("achilles", Achilles_msg, queue_size=1)
        self.pub_regist = rospy.Publisher("authority_regist", String_necst, queue_size=1)
        self.pub_obsstatus = rospy.Publisher("obs_status", Status_obs_msg, queue_size=1)
        self.pub_onestatus = rospy.Publisher("one_status", Status_onepoint_msg, queue_size=1)        
        self.pub_queue = rospy.Publisher("queue_obs", Bool_necst, queue_size=1)
        self.pub_alert = rospy.Publisher("alert", String_necst, queue_size=1)
        self.pub_txt = rospy.Publisher("text1", textfile_msg, queue_size=10)
        self.pub_analy = rospy.Publisher("auto_analy", analy_msg, queue_size=10)
        self.service_ac240 = rospy.ServiceProxy("ac240", ac240_srv)
        self.service_encoder = rospy.ServiceProxy("encoder_origin", Bool_srv)
        self.pub_log = rospy.Publisher("logging_ctrl", String, queue_size=1)
        self.pub_xffts = rospy.Publisher("XFFTS_DB_flag", xffts_flag_msg, queue_size = 1)
        self.pub_encdb = rospy.Publisher("encoder_DB_flag", encdb_flag_msg, queue_size = 1)
        self.logger_flag = rospy.Publisher("logger_path", String, queue_size=1)
        time.sleep(0.5)# authority regist time
        self.pub_beam("center_beam_num", Center_beam_num_msg, queue_size=1)

        now = datetime.utcnow()
        log_path = '/home/amigos/log/{}.log'.format(now.strftime('%Y%m%d'))
        self.logger = logger.logger(__name__, filename=log_path)
        self.log = self.logger.setup_logger()

        """get authority"""
        if not self.escape:
            self.get_authority()
        
        """finish action"""
        atexit.register(self._release)

        time.sleep(1.)# authority regist time

        self.nc = nasco_controller.controller(node=False)
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

# ==================
# Logger
# ==================

    def logger(func):
        @functools.wraps(func)
        def ros_logger(self, *args, **kwargs):
            self.pub_log.publish("{}#{}#{}".format(func.__name__, args, kwargs))
            self.log.debug("{}#{}#{}".format(func.__name__, args, kwargs))
            ret = func(self, *args, **kwargs)
            return ret
        return ros_logger
        

# ===================
# authority
# ===================

    
    def deco_check(func):
        @functools.wraps(func)
        def wrapper(self, *args,**kwargs):
            #self.get_authority()
            if self.escape:###need to fix
                ret = func(self, *args,**kwargs)
            elif self.auth == self.node_name:
                ret = func(self, *args,**kwargs)
            else:
                ret = ""
                #rospy.logwarn("This node don't have authority...")
                self.log.warn("This node don't have authority...")
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

    @logger
    def get_authority(self):
        self.registration(self.node_name)
        return
    @logger
    def release_authority(self):
        self.registration("")
        return
    @logger
    def check_my_node(self):
        return self.node_name
    @logger
    def logging_message(self, log_str):
        pass
        

# ===================
# antenna
# ===================

    @logger
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
        switch = switch.lower()
        msg.data =  switch
        msg.from_node = self.node_name
        msg.timestamp = time.time()
        if switch in ["on", "off"]:
            self.pub_drive.publish(msg)
            self.pub_contactor.publish(msg)
            print("drive : ", switch, "!!")
        else:
            #rospy.logwarn("!!bad command!!")
            self.log.warn("!!bad command!!")
            pass
        return
    
    @logger
    @deco_check
    def onepoint_move(self, x, y, coord="altaz", off_x=0, off_y=0, offcoord='altaz', hosei='hosei_230.txt',  lamda=2600, dcos=0, limit=True, rotation=True):
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
        rotation : True -->  az=240 change az=-120 
        """
        self.command_time = time.time()
        self.pub_onepoint.publish(x, y, coord, "", off_x, off_y, offcoord, hosei, lamda, dcos, limit, rotation, self.node_name, self.command_time)
        return
    #@deco_check
    @logger
    def planet_move(self, planet, off_x=0, off_y=0, offcoord="altaz", hosei="hosei_230.txt", lamda=2600, dcos=0, limit=True, rotation=True):
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
        self.pub_planet.publish(0, 0, "planet", planet, off_x, off_y, offcoord, hosei, lamda, dcos,limit, rotation, self.node_name, time.time())
        return
    
    @logger    
    @deco_check
    def linear_move(self, x, y, coord="altaz", dx=0, dy=0, offcoord='altaz', hosei='hosei_230.txt',  lamda=2600, dcos=0, limit=True, rotation=True):
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
        self.pub_linear.publish(x, y, coord, "", dx, dy, offcoord, hosei, lamda, dcos, limit, rotation, self.node_name, time.time())
        return
    
    @logger
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
        self.log.info("start OTF scan!!")

        self.pub_otf.publish(x, y, coord, dx, dy, dt, num, rampt,
                             delay, off_x, off_y, offcoord,
                             dcos, hosei, lamda, limit, self.node_name,
                             current_time)
        
        return
    
    @logger
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
        self.log.info("start OTF scan!!")
        self.pub_planet_scan.publish(0, 0, planet, dx, dy, dt, num, rampt,
                             delay, off_x, off_y, offcoord,
                             dcos, hosei, lamda, limit, self.node_name,
                             current_time)        
        return
    
    @logger
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
        #self.antenna_tracking_time = req.command_time
        return

    @logger
    def antenna_tracking_check(self):
        """antenna_tracking_check"""
        self.log.info(" tracking now...")
        time.sleep(2.)
        while not self.antenna_tracking_flag:# or (int(self.command_time) != self.antenna_tracking_time):
            time.sleep(0.01)
            pass
        return

    def _obs_stop(self, req):
        self.registration("", "master")
        self.move_stop()
        print(req.data)
        return
    
    @logger
    def move_stop(self):
        self.log.info("move_stop")
        self.pub_stop.publish(True, self.node_name, time.time())
        time.sleep(0.2)
        return

    def beam_center(self, center=1):
        self.pub_beam.publish(center)
        time.sleep(0.1)
        return

# ===================
# dome
# ===================

    @logger
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
    
    @logger
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
    
    @logger
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

    @logger
    def dome_close(self):
        """Dome close"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_close'
        dome.from_node = self.node_name
        dome.timestamp = time.time()        
        self.pub_dome.publish(dome)
        return
    
    @logger
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

    @logger
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

    @logger
    def memb_close(self):
        """membrane close"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'memb_close'
        dome.from_node = self.node_name
        dome.timestamp = time.time()        
        self.pub_dome.publish(dome)
        return
    @logger
    def dome_stop(self):
        """Dome stop"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_stop'
        dome.from_node = self.node_name
        dome.timestamp = time.time()
        self.pub_dome.publish(dome)
        return

    @logger
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
    
    @logger
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
    
    @logger
    @deco_check    
    def dome_tracking_check(self):
        """dome tracking check"""
        self.log.info(" dome_tracking now...")
        while not self.dome_tracking_flag:
            time.sleep(0.01)
            pass
        return

# ===================
# mirror
# ===================
    @logger
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
    
    @logger
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

    @logger
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

    @logger
    @deco_check
    def move_chopper(self, mode):
        """
        The Chopper rotates 90(360) degrees when the pulse setting
        is 250(1000).
        Chopper's in/out switches at 90 degrees.
        """
        if mode.lower() == "in":
            pulse = 0
        elif mode.lower() == "out":
            pulse = 250
        else:
            self.log.error("{} is not valid. Chopper mode is only 'in' or 'out'.".format(mode))
            return
        self.nc.slider0.set_step("u", pulse)
        return

# ===================
# encoder
# ===================

    @logger
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
# getting data
# ===================
#@logger
    #@deco_check    
    def oneshot_achilles(self, repeat=1, exposure=1.0, stime=0.0):
        """get spectrum by ac240

        Parameter
        ---------
        repeat : repeat number
        exposure : exposure time [s]
        stime : start mjd time [day]
        
        """
        rospy.wait_for_service("ac240")
        response = self.service_ac240(repeat, exposure, stime)
        #print(response)
        #print(len(response.dfs1))
        dfs1_list = []
        dfs2_list = []
        tmp1 = [dfs1_list.append(response.dfs1[i*16384:(i+1)*16384]) for i in range(int(len(response.dfs1)/16384))]
        tmp2 = [dfs2_list.append(response.dfs2[i*16384:(i+1)*16384]) for i in range(int(len(response.dfs2)/16384))]
        
        data_dict = {"dfs1":dfs1_list, "dfs2":dfs2_list}

        return data_dict


    def old_oneshot(self, repeat=1, exposure=1.0, stime=0.0):
        dfs = achilles.dfs()
        data = dfs.oneshot(req.repeat, req.exposure, req.stime)
        data_dict = {'dfs1': data[0], 'dfs2': data[1]}
        return data_dict

    @logger
    @deco_check    
    def spectrometer(self, exposure):
        msg = Float64()
        msg.data = exposure
        self.pub3.publish(msg)
        return
    
    @logger
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
        
    @logger
    def ccd_oneshot(self, filename, dirname="/home/amigos/data/experiment/oneshot/"):
        """shot request to CCD (ROS_camera.py)
        Parameters
        ----------
        filename : save filename
        dirname  : save directory
        """        
        msg = oneshot_msg()
        msg.filename = filename
        msg.dirname = dirname
        msg.shot_mode = "oneshot"
        pub = rospy.Publisher('oneshot', oneshot_msg, queue_size=1, latch=True)
        time.sleep(0.1)
        
        print("ccd request start")
        pub.publish(msg)
        while not os.path.exists(dirname+filename+".jpg"):
            time.sleep(0.01)
        print("ccd shot complete")

        return
    
    
    # ===================
    # status
    # ===================
    
    @logger
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
    
    @logger
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
                break
            else:
                status = ""
                time.sleep(0.1)

        return status
    
    def _write_status(self, req):
        self.status = req
        self.read_sub.unregister()
        return

    # ===================
    # others
    # ===================

    #@logger
    def alert(self, message, node_name="", emergency=False):
        msg = String_necst()
        msg.data = message
        msg.from_node = node_name
        msg.timestamp = time.time()
        self.pub_alert.publish(msg)
        if emergency:
            self.pub_obs_stop.publish(message, self.node_name, time.time())
        else:
            pass
        return

    @logger
    def encoder_origin_setting(self, mode=False):
        """ encoder origin setting

        * Parameters *
        ----------
        mode : True  -> clear_condition of z_mode is "CLS0"
               False -> clear_condition of z_mode is ""
        
        * How to use *
        1.run "encoder_origin_setting(mode=True)"
        2.Move antenna through the origin
        3.run "encoder_origin_setting(mode=False)" 
        """
        rospy.wait_for_service("encoder_origin")
        response = self.service_encoder(mode)
        if response.data == True:
            print("clear_condition is 'CLS0'.")
        else:
            print("clear_condition is ''.")
        return

    @logger
    def pub_encdb_flag(self, boolflag, dbname):
        encflag = encdb_flag_msg()
        encflag.data = boolflag
        encflag.newdb_name = dbname
        encflag.timestamp = time.time()
        self.pub_encdb.publish(encflag)        

    @logger
    def xffts_publish_flag(self, obs_mode="Non", scan_num=0, lamdel=0, betdel=0):
        xffts_flag = xffts_flag_msg()
        xffts_flag.scan_num = scan_num
        xffts_flag.obs_mode = obs_mode
        xffts_flag.lamdel = lamdel
        xffts_flag.betdel = betdel
        self.pub_xffts.publish(xffts_flag)
        pass

    def pub_txtfile(self, path, savepath):
        s = textfile_msg()
        s.data = path
        s.path = savepath
        self.pub_txt.publish(s)

    @logger
    def pub_analyexec(self, data_path, analy_type):
        s = analy_msg()
        s.data_path = data_path
        s.analy_type = analy_type
        self.pub_analy.publish(s)

    @logger
    def pub_loggerflag(self, data_path):
        s = String()
        s.data = data_path
        self.logger_flag.publish(s)
