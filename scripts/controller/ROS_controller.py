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
from necst.msg import drive_msg
from necst.msg import Velocity_mode_msg
from necst.msg import Move_mode_msg
from necst.msg import Otf_mode_msg
from necst.msg import Dome_msg
from necst.msg import Read_status_msg
from necst.msg import Achilles_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int64

class controller(object):

    task_flag = False
    antenna_tracking_flag = False
    dome_tracking_flag = False
    access_authority = "no_data"

    status = ""
    

    def __init__(self):
        rospy.init_node('controller_client')
        rospy.Subscriber("tracking_check", Bool, self.antenna_tracking)
        rospy.Subscriber("dome_tracking_check", Bool, self.dome_tracking)
        rospy.Subscriber("authority_check", String, self.authority_check)

        self.pub1 = rospy.Publisher("observation_start_", Bool, queue_size=1)#observation
        self.pub2 = rospy.Publisher("authority_change", String, queue_size = 1,latch=True)
        self.pub5 = rospy.Publisher("antenna_drive", String, queue_size = 1)
        self.pub6 = rospy.Publisher("antenna_contactor", String, queue_size = 1)
        self.pub7 = rospy.Publisher("antenna_vel", Velocity_mode_msg, queue_size = 1, latch = True)
        self.pub8 = rospy.Publisher("antenna_command", Move_mode_msg, queue_size=1, latch=True)
        self.pub9 = rospy.Publisher("assist_antenna", Move_mode_msg, queue_size=1, latch=True)
        self.pub11 = rospy.Publisher("move_stop", String, queue_size = 1, latch = True)
        self.pub12 = rospy.Publisher("antenna_otf", Otf_mode_msg, queue_size = 1, latch = True)

        self.pub13 = rospy.Publisher("dome_move", Dome_msg, queue_size = 1, latch = True)
        self.pub14 = rospy.Publisher('m4', String, queue_size = 1, latch = True)
        self.pub15 = rospy.Publisher("hot", String, queue_size = 1, latch = True)
        self.pub16 = rospy.Publisher("m2", Int64, queue_size=1, latch=True)
        self.pub17 = rospy.Publisher("achilles", Achilles_msg, queue_size=1)
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

    def antenna_tracking(self, req):
        self.antenna_tracking_flag = req.data
        return

    def antenna_tracking_check(self):
        rospy.loginfo(" tracking now... \n")
        time.sleep(3.)
        while not self.antenna_tracking_flag:
            time.sleep(0.01)
            pass

    def drive(self, switch = ""):
        """change drive"""
        if not switch:
            switch = str(input("drive change (on/off) : "))
        if switch.lower() == "on":
            self.drive_on()
            self.contactor_on()
            print("driver on!!")
        elif switch.lower() == "off":
            self.drive_off()
            self.contactor_off()
            print("driver off...")
        else:
            print("!!bad command!!")

    def drive_on(self):
        """drive_on"""
        msg = String()
        msg.data = "on"
        self.move_stop()
        self.pub5.publish(msg) 
        return

    def drive_off(self):
        """drive_off"""
        msg = String()
        msg.data = "off"
        self.move_stop()
        self.pub5.publish(msg)
        return

    def contactor_on(self):
        msg = String()
        msg.data = "on"
        self.move_stop()
        self.pub6.publish(msg)
        return

    def contactor_off(self):
        msg = String()
        msg.data = "off"
        self.move_stop()
        self.pub6.publish(msg)
        return

    def move(self, x, y, coord, planet= 0, off_x=0, off_y=0, offcoord='horizontal', hosei='hosei_230.txt',  lamda=2600, dcos=0, vel_x=0, vel_y=0, movetime=10, limit=True, assist=True):
        """ azel_move, radec_move, galactic_move, planet_move
        
        Parameters
        ----------
        x        : target_x [deg]
        y        : target_y [deg]
        coord    : "horizontal" or "j2000" or "b1950" or "galactic" or "planet" 
        planet   : planet_number (only when using "planet_move"!!)
                   1.Mercury 2.Venus 3. 4.Mars 5.Jupiter 6.Saturn 7.Uranus 8.Neptune, 9.Pluto, 10.Moon, 11.Sun
        off_x    : offset_x [deg]
        off_y    : offset_y [deg]
        offcoord : "horizontal" or "j2000" or "b1950" or "galactic" or "planet" 
        hosei    : hosei file name (default ; hosei_230.txt)
        lamda    : observation wavelength [um] (default ; 2600)
        dcos     : projection (no:0, yes:1)
        vel_x    : constant speed scan [arcsec/s] (horizontal)
        vel_y    : constant speed scan [arcsec/s] (horizontal)
        movetime : azel_list length [s]
        limit    : soft limit [az:-240~240, el:30~80] (True:limit_on, False:limit_off)
        assist   : ROS_antenna_assist is on or off (True:on, False:off)
        """
        if assist:
            self.pub8.publish(x, y, coord, planet, off_x, off_y, offcoord, hosei, lamda, dcos, vel_x, vel_y, movetime, limit, time.time())
        else:
            self.pub9.publish(x, y, coord, planet, off_x, off_y, offcoord, hosei, lamda, dcos, vel_x, vel_y, movetime, limit, time.time())
        return


    def azel_move(self, az, el, off_x = 0, off_y = 0, offcoord = 'horizontal', hosei = 'hosei_230.txt', lamda=2600, dcos=0, vel_x=0, vel_y=0, movetime=10, limit=True, assist=True):
        """
        azel_move
        More detail is move()
        """
        self.move(az, el, "horizontal", 0, off_x, off_y, offcoord, hosei, lamda, dcos, vel_x, vel_y, movetime, limit, assist)
        return

    def radec_move(self, ra, dec, coord, off_x = 0, off_y = 0, offcoord = 'horizontal', hosei = 'hosei_230.txt', lamda=2600, dcos=0, vel_x=0, vel_y=0, movetime=10, limit=True, assist=True):
        """
        radec_move
        More detail is move()
        """
        print("start radec_move!!")
        self.move(ra, dec, coord, 0, off_x, off_y, offcoord, hosei, lamda, dcos, vel_x, vel_y, movetime, limit, assist)
        return
    
    def galactic_move(self, l, b, off_x = 0, off_y = 0, offcoord = 'horizontal', hosei = 'hosei_230.txt', lamda=2600, dcos=0, vel_x=0, vel_y=0, movetime=10, limit=True, assist=True):
        """
        galactic_move
        More detail is move()
        """
        print("start galactic_move!!")
        self.move(l, b, "galactic", 0, off_x, off_y, offcoord, hosei, lamda, dcos, vel_x, vel_y, movetime, limit, assist)
        return

    def planet_move(self, number, off_x = 0, off_y = 0, offcoord = 'HORIZONTAL', hosei = 'hosei_230.txt', lamda=2600, dcos=0, vel_x=0, vel_y=0, movetime=10, limit=True, assist=True):
        """
        planet_move
        1.Mercury 2.Venus 3. 4.Mars 5.Jupiter 6.Saturn 7.Uranus 8.Neptune, 9.Pluto, 10.Moon, 11.Sun
        More detail is move()
        """
        print("start planet_move!!")
        self.move(0, 0, "planet", number, off_x, off_y, offcoord, hosei, lamda, dcos, vel_x, vel_y, movetime, limit, assist)
        return

    def move_stop(self):
        print("move_stop")
        self.pub11.publish("stop")
        time.sleep(0.2)
        return

    def otf_scan(self, x, y, coord, dx, dy, dt, num, rampt, delay, start_on,  off_x=0, off_y=0, offcoord="j2000", dcos=0, hosei="hosei_230.txt", lamda=2600., movetime=0.01, limit=True):
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
        
        print("start OTF scan!!")
        self.pub12.publish(x, y, coord, dx, dy, dt, num,rampt,
                           delay, start_on, off_x, off_y, offcoord,
                           dcos, hosei, lamda, movetime, limit)
        
        return 

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
        time.sleep(3.)
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
        day = dt.utcnow().strftime("%y%m%d_%H%M%S")
        self.pub17.publish(repeat, exposure, stime, day)
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
    #a = float(input("x : "))
    #b = float(input("y : "))
    #con.otf_scan(a, b, "j2000", 30, 0, 0.6, 9, 0.6*4, delay=10., off_x=-900, off_y=0, offcoord="j2000")
    from astropy.time import Time
    import datetime
    tt = Time(dt.utcnow() + datetime.timedelta(seconds=5)).mjd
    aa = con.oneshot_achilles(3, 1, tt)
    print(aa)
