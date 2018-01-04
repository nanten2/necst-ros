import math
import time
import datetime
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")
import ROS_controller
import signal
import sys
import ccd
from astropy.coordinate import Skycoord
from astropy.time import Time
from datetime import datetime as dt
sys.path.append('/home/amigos/ros/src/necst/lib')
import azel_calc


class opt_point_controller(object):
    #reference : ccd.py, imageppm.cpp, imagepgm.cpp
    
    
    pointing_list = "/home/amigos/NECST/soft/core/pointing.list"
    tai_utc = 36.0 # tai_utc=TAI-UTC  2015 July from ftp://maia.usno.navy.mil/ser7/tai-utc.dat
    dut1 = 0.14708
    
    
    def __init__(self):
        self.ctrl = ROS_controller.controller()
        self.calc = azel_calc.azel_calc()
        return
    
    def handler(self, num, flame):
        print("!!ctrl+C!!")
        print("STOP MOVING")
        self.ctrl.move_stop()
        sys.exit()


    """
    def calc_star_azel(self, ra, dec, mjd):
        ra = ra*math.pi/180.
        dec = dec*math.pi/180.
        
        ret = slalib.sla_map(ra, dec, 0, 0, 0, 0, 2000, mjd + (self.tai_utc + 32.184)/(24.*3600.))
        ret = list(ret)
        ret = slalib.sla_aop(ret[0], ret[1], mjd, self.dut1, -67.70308139*math.pi/180, -22.96995611*math.pi/180, 4863.85, 0, 0, 283, 500, 0.1, 0.5, tlr=0.0065)
        real_az = ret[0]
        real_el = math.pi/2. - ret[1]
           
        real_az = real_az*180./math.pi
        real_el = real_el*180./math.pi
        return [real_az, real_el]
        """

    def create_table(self):
        #create target_list
        
        f = open(self.pointing_list)
        line = f.readline()
        target_list = []
        
        #calculate mjd(now) and mjd(2000)
        now = dt.now()
        _date = Time(now).mjd
        
        while line:
            list = []
            line = line.replace(";", " ")
            line = line.split()
            
            #number(FK6)
            list.append(line[0])
            
            #ra,dec(degree)
            now = dt.now()
            _date = Time(now).mjd
            coord = SkyCoord(str(line[1])+'h'+ str(line[2]) +'m'+ str(line[3]) +'s',str(line[5]) + str(line[6]) + 'd' + str(line[7]) + 'm' + str(line[8]) + 's', frame='icrs')
            ra = coord.ra.deg + float(line[4])*(360./24.)/3600.*(_date - 51544)/36525.
            dec = coord.dec.deg + float(line[9])*(360./24.)/3600.*(_date - 51544)/36525.
            
            list.append(ra)
            list.append(dec)
            list.append(line[21]) #magnitude
            
            ret = self.calc.coordinate_calc(ra, dec, 0, "j2000", 0, 0, off_x=0, off_y=0, offcoord="horizontal", hosei="hosei_230.txt", lamda=2600, dcos=1, temp=20, press=5, humi=0.07, now=now, loop = 1, time_rate=0.)
            list.append(ret[0]) #az
            #list = [number, ra, dec, magnitude, az]
            
            #print(ret[1])
            print(str(ra)+"  "+str(dec))
            
            if ret[1] >= 30 and ret[1] <= 80:
                print("============")
                num = len(target_list)
                if num == 0:
                    target_list.append(list)
                elif num == 1:
                    if target_list[0][4] < list[4]:
                        target_list.append(list)
                    else:
                        target_list.insert(0, list)
                else:
                    for i in range(num):
                        if target_list[i][4] > list[4]:
                            target_list.insert(i, list)
                            break
                        if i == num-1:
                            target_list.insert(num, list)
            
            #print(target_list)
            line = f.readline()
        
        f.close()
        return target_list
    
    def start_observation(self):
        signal.signal(signal.SIGINT, self.handler)
        table = self.create_table()
        num = len(table)
        
        #self.ctrl.dome_track()#test
        print(table)
        
        date = dt.today()
        month = str("{0:02d}".format(date.month))
        day = str("{0:02d}".format(date.day))
        hour = str("{0:02d}".format(date.hour))
        minute = str("{0:02d}".format(date.minute))
        second = str("{0:02d}".format(date.second))
        data_name = "opt_"+str(date.year)+month+day+hour+minute+second
        
        
        for _tbl in table:
            
            #calculate Az and El for check
            #ret = self.calc_star_azel(_tbl[1], _tbl[2], mjd2)
            ret = self.calc.coordinate_calc(_tbl[1],_tbl[2] 0, "j2000", 0, 0, off_x=0, off_y=0, offcoord="horizontal", hosei="hosei_230.txt", lamda=2600, dcos=1, temp=20, press=5, humi=0.07, now=now, loop = 1, time_rate=0.)
            real_el = ret[1]

            '''test
            if real_el >= 30. and real_el <= 80.:
                self.ctrl.radec_move(_tbl[1], _tbl[2], "J2000", 0, 0, hosei = 'hosei_opt.txt', offcoord = 'HORIZONTAL', lamda = 0.5)
                print(_tbl[1], _tbl[2])
                print(ret)
                
                self.ctrl.antenna_tracking_check()
                self.ctrl.dome_tracking_check()
                
                now = dt.now()
                #n_star = self.calc_star_azel(_tbl[1], _tbl[2], _date)
                ret = self.calc.coordinate_calc(_tbl[1],_tbl[2] 0, "j2000", 0, 0, off_x=0, off_y=0, offcoord="horizontal", hosei="hosei_230.txt", lamda=2600, dcos=1, temp=20, press=5, humi=0.07, now=now, loop = 1, time_rate=0.)
                ccd.all_sky_shot(_tbl[0], _tbl[3], ret[0], ret[1], data_name, status)
            else:
                #out of range(El)
                pass
                '''

        self.ctrl.move_stop()
        print("OBSERVATION END")
        return
    
    
