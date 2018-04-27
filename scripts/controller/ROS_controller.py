#! /usr/bin/env python3
# coding:utf-8

"""
------------------------------------------------
[History]
2017/10/18 : kondo takashi
2018/01/16 : kondo
------------------------------------------------
"""
import os
import sys
import time
from datetime import datetime as dt
import rospy
#from necst.msg import drive_msg
from necst.msg import Move_mode_msg
from necst.msg import Otf_mode_msg
from necst.msg import Dome_msg
from necst.msg import Read_status_msg
from necst.msg import Achilles_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
#from std_msgs.msg import Float64
from std_msgs.msg import Int64
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
        self.name = auth.initialize()
        rospy.init_node(self.name)
        auth.start()
        
        """init"""
        rospy.Subscriber("tracking_check", Bool, self.antenna_tracking)
        rospy.Subscriber("dome_tracking_check", Bool, self.dome_tracking)

        self.pub_drive = rospy.Publisher("antenna_drive", String, queue_size = 1)
        self.pub_contactor = rospy.Publisher("antenna_contactor", String, queue_size = 1)
        self.pub_onepoint = rospy.Publisher("onepoint_command", Move_mode_msg, queue_size=1, latch=True)
        self.pub_planet = rospy.Publisher("planet_command", Move_mode_msg, queue_size=1, latch=True)        
        self.pub_stop = rospy.Publisher("move_stop", Bool, queue_size = 1, latch = True)
        self.pub_otf = rospy.Publisher("antenna_otf", Otf_mode_msg, queue_size = 1, latch = True)
        self.pub_planet_scan = rospy.Publisher("planet_otf", Otf_mode_msg, queue_size = 1, latch = True)        
        self.pub_dome = rospy.Publisher("dome_move", Dome_msg, queue_size = 1, latch = True)
        self.pub_m4 = rospy.Publisher('m4', String, queue_size = 1, latch = True)
        self.pub_hot = rospy.Publisher("hot", String, queue_size = 1, latch = True)
        self.pub_m2 = rospy.Publisher("m2", Int64, queue_size=1, latch=True)
        self.pub_achilles = rospy.Publisher("achilles", Achilles_msg, queue_size=1)
        time.sleep(0.5)

        return
    
    def get_authority(self):
        auth.registration(self.name)
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
    @auth.deco_check
    def drive(self, switch = ""):
        """change drive

        Parameters
        ----------
        swith : on or off
        
        """
        self.move_stop()
        if not switch:
            switch = str(input("drive change (on/off) : "))
        command =  switch.lower()
        if command == "on" or command == "off":
            self.pub_drive.publish(command)
            self.pub_contactor.publish(command)
            print("drive : ", command, "!!")
        else:
            print("!!bad command!!")

    def onepoint_move(self, x, y, coord="horizontal", off_x=0, off_y=0, offcoord='horizontal', hosei='hosei_230.txt',  lamda=2600, dcos=0, func_x="", func_y="", movetime=10, limit=True,):
        """ azel_move, radec_move, galactic_move
        
        Parameters
        ----------
        x        : target_x [deg]
        y        : target_y [deg]
        coord    : "horizontal" or "j2000" or "b1950" or "galactic"  
        off_x    : offset_x [arcsec]
        off_y    : offset_y [arcsec]
        offcoord : "horizontal" or "j2000" or "b1950" or "galactic" 
        hosei    : hosei file name (default ; hosei_230.txt)
        lamda    : observation wavelength [um] (default ; 2600)
        dcos     : projection (no:0, yes:1)
        func_x   : free scan [arcsec/s] (cf:20*x or math.sin(x) or etc...)
        func_y   : free scan [arcsec/s] (cf:20*y or math.sin(y) or etc...)
        movetime : azel_list length [s]
        limit    : soft limit [az:-240~240, el:30~80] (True:limit_on, False:limit_off)
        assist   : ROS_antenna_assist is on or off (True:on, False:off)
        """
        self.pub_stop.publish(False)
        self.pub_onepoint.publish(x, y, coord, 0, off_x, off_y, offcoord, hosei, lamda, dcos, str(func_x), str(func_y), limit, self.name, time.time())
        return

    def planet_move(self):
        """ planet_move
        
        Parameters
        ----------
        planet   : planet_number (only when using "planet_move"!!)
                   1.Mercury 2.Venus 3. 4.Mars 5.Jupiter 6.Saturn 7.Uranus 8.Neptune, 9.Pluto, 10.Moon, 11.Sun
        off_x    : offset_x [arcsec]
        off_y    : offset_y [arcsec]
        offcoord : "horizontal" or "j2000" or "b1950" or "galactic" 
        hosei    : hosei file name (default ; hosei_230.txt)
        lamda    : observation wavelength [um] (default ; 2600)
        dcos     : projection (no:0, yes:1)
        func_x   : free scan [arcsec/s] (cf:20*x or math.sin(x) or etc...)
        func_y   : free scan [arcsec/s] (cf:20*y or math.sin(y) or etc...)
        movetime : azel_list length [s]
        limit    : soft limit [az:-240~240, el:30~80] (True:limit_on, False:limit_off)
        assist   : ROS_antenna_assist is on or off (True:on, False:off)
        """
        if isinstance(planet, str):
            planet_list = {"mercury":1, "venus":2, "mars":4, "jupiter":5,"saturn":6, "uranus":7, "neptune":8, "moon":10, "sun":11}
            planet = planet_list[planet.lower()]
        else:
            pass
        self.pub_stop.publish(False)
        self.pub_planet.publish(0, 0, coord, planet, off_x, off_y, offcoord, hosei, lamda, dcos, str(func_x), str(func_y), movetime, limit, time.time())
        return
        
        pass

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
        self.pub_stop.publish(False)
        self.pub_otf.publish(x, y, coord, dx, dy, dt, num, rampt,
                             delay, start_on, off_x, off_y, offcoord,
                             dcos, hosei, lamda, limit, self.name,
                             current_time)
        
        return 

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
        self.pub_stop.publish(False)
        self.pub_planet_scan.publish(x, y, coord, dx, dy, dt, num,rampt,
                                      delay, start_on, off_x, off_y, offcoord,
                                      dcos, hosei, lamda, movetime, limit,
                                      assist,time.time())
        

    
    def move_stop(self):
        print("move_stop")
        self.pub_stop.publish(True)
        time.sleep(0.2)
        return
        
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
        self.pub_dome.publish(dome)

    def dome_open(self):
        """Dome open"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_open'
        self.pub_dome.publish(dome)
    
    def dome_close(self):
        """Dome close"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_close'
        self.pub_dome.publish(dome)

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

        
    def memb_open(self):
        """membrane open"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'memb_open'
        self.pub_dome.publish(dome)

    def memb_close(self):
        """membrane close"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'memb_close'
        self.pub_dome.publish(dome)
        
    def dome_stop(self):
        """Dome stop"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_stop'
        self.pub_dome.publish(dome)
        
    def dome_track(self):
        """Dome sync antenna_az"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_tracking'
        self.pub_dome.publish(dome)

    def dome_track_end(self):
        """Dome stop antenna_az sync"""
        dome = Dome_msg()
        dome.name = 'command'
        dome.value = 'dome_track_end'
        self.pub_dome.publish(dome)

    def dome_tracking(self, req):
        self.dome_tracking_flag = req.data
        return

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

    def move_m4(self, position):
        """mirror4 move
        
        Parameter
        ---------
        position : in or out
        
        """
        status = String()
        status.data = position
        self.pub_m4.publish(status)
        return

    
    def move_hot(self, position):
        """hotload move

        Parameter
        ---------
        position : in or out
        """
        status = String()
        status.data = position
        self.pub_hot.publish(status)
        return

    def m2_move(self, dist):
        """m2 move

        Parameter
        ---------
        dist : distance [um]
        
        """
        status = Int64()
        status.data = dist
        self.pub_m2.publish(status)
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
        """get spectrum by ac240

        Parameter
        ---------
        repeat : repeat number
        exposure : exposure time [s]
        stime : start mjd time [day]
        
        """
        day = dt.utcnow().strftime("%y%m%d_%H%M%S")
        self.pub_achilles.publish(repeat, exposure, stime, day)
        dir_name = "/home/amigos/data/experiment/achilles/" + str(day) + "/"
        file_name = day + "_fin.txt"
        while not rospy.is_shutdown():
            if not os.path.exists(dir_name + file_name):
                print("get data now...")
                pass
            else:
                break
            time.sleep(1)
        with open(dir_name+day + "_1.txt", "r") as f1:
            dfs1 = f1.readline()
        with open(dir_name+day + "_2.txt", "r") as f2:
            dfs2 = f2.readline()
        data_dict = {'dfs1': eval(dfs1), 'dfs2': eval(dfs2)}
        return data_dict

    def spectrometer(self, exposure):
        msg = Float64()
        msg.data = exposure
        self.pub3.publish(msg)
        return

# ===================
# status
# ===================

    def read_status(self):
        """read status

        attention!!
        -----------
        you need execution this script.

        $rosrun necst ROS_status.py 1
        or 
        $python ROS_status.py 1

        """
        self.sub = rospy.Subscriber("read_status", Read_status_msg, self.write_status)
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
        self.sub.unregister()
        return

if __name__ == "__main__":
    con = controller()
    a = float(input("x : "))
    b = float(input("y : "))
    from datetime import datetime as dt
    import time
    utc = dt.utcnow()
    utc_list = [utc.year,utc.month,utc.day,utc.hour,utc.minute,utc.second,utc.microsecond]
    #con.otf_scan(a, b, "j2000", 30, 0, 0.6, 9, 0.6*4, delay=10., start_on=utc_list, off_x=-900, off_y=0, offcoord="j2000")
    con.otf_scan(-30, 20, "j2000", 30, 0, 0.6, 9, 0.6*4, 3, start_on=0,  off_x=0, off_y=0, offcoord="j2000", dcos=0, hosei="hosei_230.txt", lamda=2600., limit=True)
