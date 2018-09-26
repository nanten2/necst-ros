#!/usr/bin/env python

import math
import time
import datetime
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")
import opt_analy
import ROS_controller
import signal
import sys
import ccd_old as ccd
from astropy.coordinates import SkyCoord,EarthLocation
from astropy.time import Time
from datetime import datetime as dt
import astropy.units as u
sys.path.append('/home/amigos/ros/src/necst/lib')
#import azel_calc
import calc_coord
nanten2 = EarthLocation(lat=-22.9699511*u.deg, lon=-67.60308139*u.deg, height=4863.84*u.m)


""" 
Notes about opt_point.py
------------------------
opt_point.py is used for optical pointing.
From the pointing list(pointing.list), select an observable object and move the antenna there with equatorial coordinate(J2000/FK5)
"""

class opt_point_controller(object):
    #reference : ccd.py, imageppm.cpp, imagepgm.cpp
    
    
    #pointing_list = "/home/amigos/NECST/soft/core/pointing.list"
    pointing_list = "/home/amigos/ros/src/necst/lib/pointing.list"
    pointing_list2 = "/home/necst/ros/src/necst/lib/pointing.list"
    tai_utc = 36.0 # tai_utc=TAI-UTC  2015 July from ftp://maia.usno.navy.mil/ser7/tai-utc.dat
    dut1 = 0.14708
    
    
    def __init__(self):
        self.ctrl = ROS_controller.controller()
        #self.calc = azel_calc.azel_calc()
        self.calc = calc_coord.azel_calc()
        self.ctrl.obs_status(active=True, obsmode="All_sky_shot", obs_script="all_sky_shot.py", obs_file="", target="fourth magnitude star")
        return
    
    def handler(self, num, flame):
        print("!!ctrl+C!!")
        print("STOP MOVING")
        self.ctrl.move_stop()
        self.ctrl.dome_stop()
        self.ctrl.obs_status(active=False)
        time.sleep(3.)
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

    def create_table(self,sort = 'az'):
        """
        Returns
        -------
        target_list : list
            observable object list
        target_list:
        [0] : star number
        [1] : ra
        [2] : dec
        [3] : star magnitude
        [4] : azimuth
        """

        #create target_list
        try:
            f = open(self.pointing_list)
        except:
            f = open(self.pointing_list2)
        line = f.readline()
        target_list = []
        
        #calculate mjd(now) and mjd(2000)
        now = dt.now()
        _date = Time(now).mjd
        
        while line:
            _list = []
            line = line.replace(";", " ")
            line = line.split()
            
            #number(FK6)
            _list.append(line[0])
            
            #ra,dec(degree)
            now = dt.now()
            _date = Time(now).mjd
            coord = SkyCoord(str(line[1])+'h'+ str(line[2]) +'m'+ str(line[3]) +'s',str(line[5]) + str(line[6]) + 'd' + str(line[7]) + 'm' + str(line[8]) + 's', frame='icrs')
            ra = coord.ra.deg + float(line[4])*(360./24.)/3600.*(_date - 51544)/36525.
            dec = coord.dec.deg + float(line[9])*(360./24.)/3600.*(_date - 51544)/36525.
            
            _list.append(ra)
            _list.append(dec)
            _list.append(line[21]) #magnitude
            
            #ret = self.calc.coordinate_calc(ra, dec, "j2000", 0, off_x=0, off_y=0, offcoord="horizontal", hosei="hosei_230.txt", lamda=2600, dcos=1, temp=20, press=5, humi=0.07, now=now, movetime=0.1)

            #store parameters in lists to use self.calc.coordinate_calc
            ra = [ra*3600.]
            dec = [dec*3600.]
            now = [now]
            print(ra,dec,now)
                        
            ret = self.calc.coordinate_calc(ra, dec, now, 'fk5', 0, 0, 'hosei_opt.txt', 2600, 5, 20, 0.07)
            _list.append(ret[0][0]) #az arcsec
            _list.append(ret[1][0])
            #list = [number, ra, dec, magnitude, az]
            #print(str(ra)+"  "+str(dec))
            if _list[4] > 3600*180:#
                _list[4] = _list[4] -3600*360
            
            if sort == 'az' or sort == "r_az":
                if ret[1][0]/3600. >= 30 and ret[1][0]/3600. < 80:
                    print("============")
                    num = len(target_list)
                    if num == 0:
                        target_list.append(_list) 
                    elif num == 1:
                        if target_list[0][4] < _list[4]:
                            target_list.append(_list)
                        else:
                            target_list.insert(0, _list)
                    else:
                        for i in range(num):
                            if target_list[i][4] > _list[4]:
                                target_list.insert(i, _list)
                                break
                            if i == num-1:
                                target_list.insert(num, _list)
                                pass

            elif sort == 'line_az':
                if ret[1][0]/3600. >= 35 and ret[1][0]/3600. <= 55:
                    print("============")
                    num = len(target_list)
                    if num == 0:
                        target_list.append(_list) 
                    elif num == 1:
                        if target_list[0][4] < _list[4]:
                            target_list.append(_list)
                        else:
                            target_list.insert(0, _list)
                    else:
                        for i in range(num):
                            if target_list[i][4] > _list[4]:
                                target_list.insert(i, _list)
                                break
                            if i == num-1:
                                target_list.insert(num, _list)
                                pass                            
                            
            elif sort == 'line_el':
                if not (-10*3600. <= _list[4] <= +10*3600.):
                    pass
                elif ret[1][0]/3600. >= 30 and ret[1][0]/3600. < 80:
                    print("============")
                    num = len(target_list)
                    if num == 0:
                        target_list.append(_list) 
                    elif num == 1:
                        if target_list[0][5] < _list[5]:
                            target_list.append(_list)
                        else:
                            target_list.insert(0, _list)
                    else:
                        for i in range(num):
                            if target_list[i][5] > _list[5]:
                                target_list.insert(i, _list)
                                break
                            if i == num-1:
                                target_list.insert(num, _list)
                                pass

            else:#el_sort
                if ret[1][0]/3600. >= 30 and ret[1][0]/3600. < 80:
                    print("============")
                    num = len(target_list)
                    if num == 0:
                        target_list.append(_list) 
                    elif num == 1:
                        if target_list[0][5] < _list[5]:
                            target_list.append(_list)
                        else:
                            target_list.insert(0, _list)
                    else:
                        for i in range(num):
                            if target_list[i][5] > _list[5]:
                                target_list.insert(i, _list)
                                break
                            if i == num-1:
                                target_list.insert(num, _list)
                                pass                            
            
            line = f.readline()
            
        if sort == "r_az":
            re_az = [i for i in reversed(target_list)]
            target_list = re_az
        f.close()
        return target_list
    
    def start_observation(self, sort = 'az'):
        signal.signal(signal.SIGINT, self.handler)
        table = self.create_table(sort = sort)
        print("#################",table)
        num = len(table)
        
        self.ctrl.dome_track()
        
        date = dt.today()
        month = str("{0:02d}".format(date.month))
        day = str("{0:02d}".format(date.day))
        hour = str("{0:02d}".format(date.hour))
        minute = str("{0:02d}".format(date.minute))
        second = str("{0:02d}".format(date.second))
        data_name = "opt_"+str(date.year)+month+day+hour+minute+second#real
        #data_name = "/home/amigos/s_opt/image/"

        
        
        for _tbl in table:
            print("table",table)
            now = dt.utcnow()
            
             #store parameters in lists to use self.calc.coordinate_calc
            __ra = [_tbl[1]*3600.]
            __dec = [_tbl[2]*3600.]
            __now = [now]

            ret = self.calc.coordinate_calc(__ra, __dec, __now, 'fk5', 0, 0, 'hosei_opt.txt', 0.5, 980, 260, 0.07)
            real_el = ret[1][0]/3600.
            print('#L161',ret)
            if real_el >= 30. and real_el < 79.5:
                #temp_coord = SkyCoord(_tbl[1], _tbl[2], frame="fk5", unit="deg")
                #temp_coord.location = nanten2
                #temp_coord.obstime = Time(dt.utcnow())
                #azel = temp_coord.altaz
                #self.ctrl.onepoint_move(azel.az.deg, azel.alt.deg, "altaz", -5700, -6100, "altaz", lamda=0.5)
                self.ctrl.onepoint_move(_tbl[1], _tbl[2], "fk5",hosei="hosei_opt.txt",lamda = 0.5, rotation = False)#lamda = 0.5 => 500

                #stop moving antenna and dome tracking
                self.ctrl.antenna_tracking_check()
                self.ctrl.dome_tracking_check()
                
                now = dt.now()
                status = self.ctrl.read_status()
                #n_star = self.calc_star_azel(_tbl[1], _tbl[2], _date)
                ret = self.calc.coordinate_calc(__ra, __dec, __now, 'fk5', 0, 0, 'hosei_opt.txt', 2600, 5, 20, 0.07)
                #__x = [_tbl[0]]
                #__y = [_tbl[3]]
                #__now = [dt.utcnow()]
                print('###ret###',ret)
                try:
                    ccd.ccd_controller().all_sky_shot(_tbl[0], _tbl[3], ret[0][0]/3600., ret[1][0]/3600., data_name, status)
                except Exception as e:
                    print(e)
                    self.ctrl.move_stop()
                    time.sleep(3)
                    sys.exit()
            else:
                #out of range(El)
                pass


        self.ctrl.move_stop()
        time.sleep(3.)

        ###plot Qlook
        ###==========
        optdata_dir = '/home/nfs/necopt-old/ccd-shot/data/'
        try:
            print('Analysis ...')
            opt_analy.opt_plot([optdata_dir+data_name], savefig=True, figname=data_name, interactive=True)
        except Exception as e:
            print(e)

        try:
            import glob
            date = dataname[:8]
            file_list = glob.glob('{}{}*'.format(optdata_dir, date))
            opt_analy.opt_plot(file_list, savefig=True, interactive=True)     
            pass
        except Exception as e:
            print(e)
        ###==========
            
        print("OBSERVATION END")
        self.ctrl.obs_status(active=False)
        return
    
    
