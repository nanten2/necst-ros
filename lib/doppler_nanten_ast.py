#! /usr/bin/env python3
#-*- coding: utf-8 -*-

#import csv
#from pyslalib import slalib
from astropy.coordinates import SkyCoord
from astropy.coordinates import FK5
from astropy.coordinates import EarthLocation
from astropy.time import Time
import astropy.units as u
import astropy._erfa as erfa
import math
import time
import sys
sys.path.append("/home/amigos/NECRX_system/base_param")
#import SG

"""
class doppler(object):

    def set_track(self, stime, x, y, coord, offset_x, offset_y, offset_dcos, offset_coord, vlsrs):
        pass

    def callibrate_doppler(self, x, y, coord, offset_x=0, offset_y=0,
                           offset_dcos=1, offset_coord="SAME",
                           vlsrs=0., stime=0):
        #ドップラーシフトの補正

        vobs = self.doppler.set_track(x, y, coord, vlsrs, offset_x, offset_y,
                                      offset_dcos, offset_coord, stime)
        return vobs[0], vobs[1], vobs[2]
"""
class doppler_nanten (object):


    #PATH_DEVICE_TABLE = "/home/amigos/NECST/soft/obs/params/device_table.prm"
    #doppler_1p85.py,motor_command.c,motor_server.c,nanten_astro.h,calc_doppler.cpp,
    dic1 = {"bandnum":2,
            #set sg_power [dBm]
            "power_sg21":13.0,
            "power_sg22":13.0,
            #light speed [km/sec]
            "LIGHT_SPEED":299792.458 }

    """
    dic1 = {"bandnum":2,
            #set frequency [GHz]
            "restFreq1":230.5380,
            "restFreq2":220.3986765,
            #12CO Rest frequency [GHz] from Koln Univ.
            "REST_FREQ_12COJ1_0":115.2712018,
            "REST_FREQ_12COJ2_1":230.5380000,
            "REST_FREQ_12COJ3_2":345.7959899,
            "REST_FREQ_12COJ4_3":461.0407682,
            "REST_FREQ_12COJ7_6":806.6518060,
            #13CO Rest frequency [GHz] from Koln Univ.
            "REST_FREQ_13COJ1_0":110.2013541,
            "REST_FREQ_13COJ2_1":220.3986190,
            "REST_FREQ_13COJ3_2":330.5878655,
            "REST_FREQ_13COJ4_3":440.7650398,
            "REST_FREQ_13COJ7_6":771.1838856,
            #C18O Rest frequency [GHz] from Koln Univ.
            "REST_FREQ_C18OJ1_0":109.7821734,
            "REST_FREQ_C18OJ2_1":219.5603541,
            "REST_FREQ_C18OJ3_2":329.3305525,
            "REST_FREQ_C18OJ4_3":439.0887658,
            "REST_FREQ_C18OJ7_6":768.2515933,
            #CI Rest frequency [GHz] from Koln Univ.
            "REST_FREQ_CI3P1_0":492.1606510,
            "REST_FREQ_CI3P2_1":809.3419700,
            #set speed [km/sec]
            #"vlsr":0,
            #Upper ***sb*:1 , Lower ***sb*:-1
            "1stsb1":1,
            "1stsb2":-1,
            #set frequency [GHz]
            "2ndLO1":8.038000000000,
            "2ndLO2":9.301318999999,

            #set sg_power [dBm]
            "power_sg21":13.0,
            "power_sg22":13.0,
            #light speed [km/sec]
            "LIGHT_SPEED":299792.458 }
    """


    coord_dict = {"equatorial" : 1,
                  "j2000"     : 1,
                  "b1950"     : 2,
                  "lb"        : 3,
                  "galactic"  : 3,
                  "gal"  : 3,
                  #"APPARENT"  : 10,
                  #"HORIZONTAL": 100,
                  "same"      : 0}
    
    #time
    t = Time.now()



    def __init__(self):
        #sys.path.append("/home/amigos/RX_system/base_param")

        #self.sg2if1 = SG.secondsg01()
        #self.sg2if2 = SG.secondsg02()

        #use device_table
        """
        for record in csv.reader(open(PATH_DEVICE_TABLE,"r")):
            r_dict[record[0]] =record[1]
            continue
        """
        pass

    #def set_track(self, x, y, coord, offset_x, offset_y, offset_dcos, offset_coord, stime):
    def set_track(self, x, y, vlsr, coord, offset_x, offset_y, offset_dcos, offset_coord, stime, restFreq1, restFreq2, firstsb1, firstsb2, secondLO1, secondLO2):
        """
        setting 2ndLO
        """
        t_now = Time.now().unix
        self.t = Time(t_now + stime, format = "unix")
        vobs = self.get_vobs(math.radians(x), math.radians(y), coord,
                             offset_x, offset_y, offset_dcos, offset_coord)
        c = self.dic1["LIGHT_SPEED"]
        for band in range(1, self.dic1["bandnum"]+1):
            if band == 1:
                #vdiff = vobs - self.dic1["vlsr"]
                vdiff = vobs - vlsr
                fdiff = vdiff / c * restFreq1
                freq21 = secondLO1 + firstsb1 * fdiff
                power21 = self.dic1["power_sg21"]
                #print('vdiff',vdiff,type(vdiff),'fdiff',fdiff,type(fdiff),'secondLO1',secondLO1,type('secondLO1'),'firstsb1',firstsb1,type(firstsb1),'self.dic1["bandnum"]',self.dic1["bandnum"],type(self.dic1["bandnum"]),'band',band,type(band))
                #print("freq21", freq21,type(freq21), "power21", power21,type(power21))
                #self.sg2if1.set_sg(freq21, power21)
                vdiff_21 = vdiff
                fdiff_21 = fdiff
                flocal_21 = freq21
                time.sleep(1.)

            elif band == 2:
                #vdiff = vobs - self.dic1["vlsr"]
                vdiff = vobs - vlsr
                fdiff = vdiff / c * restFreq2
                freq22 = secondLO2 + firstsb2 * fdiff
                power22 = self.dic1["power_sg22"]
                #print('vdiff',vdiff,type(vdiff),'fdiff',fdiff,type(fdiff),'secondLO1',secondLO1,type('secondLO1'),'firstsb1',firstsb1,type(firstsb1),'self.dic1["bandnum"]',self.dic1["bandnum"],type(self.dic1["bandnum"]),'band',band,type(band))
                #print("freq22", freq22,type(freq22), "power22", power22,type(power21))
                #self.sg2if2.set_sg(freq22, power22)
                vdiff_22 = vdiff
                fdiff_22 = fdiff
                flocal_22 = freq22
                time.sleep(1.)

        Vdiff = {"sg21":vdiff_21, "sg22":vdiff_22}
        Fdiff = {"sg21":fdiff_21, "sg22":fdiff_22}
        Flocal =  {"sg21":flocal_21, "sg22":flocal_22}
        #print("vobs=",vobs,"Vdiff=",Vdiff,"Fdiff=",Fdiff,"Flocal=",Flocal)
        return vobs,Vdiff,Fdiff,Flocal

    def t_set_track(self):
        print("if1")
        aaa = 8.038000000000
        bbb = self.dic1["power_sg21"]
        print("aaa=",aaa,"bbb=",bbb)
        #self.sg2if1.set_sg(aaa,bbb)


    def set_track_old(self, x, y, vlsr, coord, offset_x, offset_y, offset_dcos, offset_coord, stime, restFreq1, restFreq2, firstsb1, firstsb2, secondLO1, secondLO2, mjd, secofday):
        """
        setting 2ndLO
        """
        mjd = mjd + secofday / (24.* 3600.)
        vobs = self.get_vobs(math.radians(x),math.radians(y),coord,
                             offset_x, offset_y, offset_dcos, offset_coord)
        c = self.dic1["LIGHT_SPEED"]
        for band in range(1, self.dic1["bandnum"]+1):
            if band == 1:
                #vdiff = vobs - self.dic1["vlsr"]
                vdiff = vobs - vlsr
                fdiff = vdiff / c * restFreq1
                freq21 = secondLO1 + firstsb1 * fdiff
                power21 = self.dic1["power_sg21"]
                #print("freq21", freq21, "power21", power21)
                #self.sg2if1.set_sg(freq21, power21)
                vdiff_21 = vdiff
                fdiff_21 = fdiff
                flocal_21 = freq21
                time.sleep(1.)

            elif band == 2:
                #vdiff = vobs - self.dic1["vlsr"]
                vdiff = vobs - vlsr
                fdiff = vdiff / c * restFreq2
                freq22 = secondLO2 + firstsb2 * fdiff
                power22 = self.dic1["power_sg22"]
                #print("freq22", freq22, "power22", power22)
                #self.sg2if2.set_sg(freq22, power22)
                vdiff_22 = vdiff
                fdiff_22 = fdiff
                flocal_22 = freq22

        Vdiff = {"sg21":vdiff_21, "sg22":vdiff_22}
        Fdiff = {"sg21":fdiff_21, "sg22":fdiff_22}
        Flocal =  {"sg21":flocal_21, "sg22":flocal_22}
        #print("vobs=",vobs,"Vdiff=",Vdiff,"Fdiff=",Fdiff,"Flocal=",Flocal)
        return vobs,Vdiff,Fdiff,Flocal
    """
    def get_status(self):
        freq_sg = self.sg.freq_check()

        pow_sg =

        freq = {"sg":freq_sg, }
        power = {"sg":pow_sg,}
        return {"freq":freq, "power":power}
    """
    def get_vobs(self, xtmp, ytmp, mode, offx, offy, dcos, offmode):
        #mode convert to j2000
        mode = mode.lower()
        offmode = offmode.lower()
        mode = self.coord_dict[mode]
        offmode = self.coord_dict[offmode]
        if offmode == self.coord_dict["same"]:
            ytmp += offy
            if dcos == 0 :
                xtmp += offx
            else :
                xtmp += offx/math.cos(ytmp)
        elif offmode == mode :
            ytmp += offy
            if dcos == 0 :
                xtmp += offx
            else :
                xtmp += offx/math.cos(ytmp)
        else :
            print("error coord != off_coord")
            pass
        if mode == self.coord_dict["j2000"] :
            xxtmp = xtmp
            yytmp = ytmp
        elif mode == self.coord_dict["b1950"] :
            sc_fk4 = SkyCoord( ra = xtmp*u.radian, dec = ytmp*u.radian, frame = "fk4")
            sc_fk5 = sc_fk4.fk5
            ret = [sc_fk5.ra.radian, sc_fk5.dec.radian]
            xxtmp = ret[0]
            yytmp = ret[1]
        elif mode == self.coord_dict["lb"] or mode == self.coord_dict["galactic"] or mode == self.coord_dict["gal"]:
            sc_gal = SkyCoord( ra = xtmp*u.radian, dec = ytmp*u.radian, frame = "galactic")
            sc_fk5 = sc_gal.fk5
            ret = [sc_fk5.ra.radian, sc_fk5.dec.radian]
            xxtmp = ret[0]
            yytmp = ret[1]
        else :
            xxtmp = xtmp
            yytmp = ytmp
        print(xxtmp, yytmp)
        print(xxtmp/3600.)
        print(xxtmp*360./2*3.14)
        vobs = self.calc_vobs(xxtmp, yytmp)
        print('vobs',vobs,type(vobs))
        return vobs

    def calc_vobs(self, ra_2000, dec_2000):
        
        x_2000 = []
        x = []
        x_solar = []
        v_earth = []
        v_solar = []
        solar_tmp = []

        #1.85m at nobeyama
        #glongitude = 138.472153
        #nanten2 at atacama (nanten2wiki)
        glongitude = -67.70308139
        #1.85m at nobeyama
        #glatitude = 35.940874
        #nanten2 at atacama (nanten2wiki)
        glatitude = -22.96995611 
        #1.85m at nobeyama
        #gheight = 1386
        #nanten2 at atacama
        gheight = 4863.85

        #init
        a = math.cos(dec_2000)
        x_2000 = [a * math.cos(ra_2000), a * math.sin(ra_2000), math.sin(dec_2000)]

        tu = (self.t.jd - 2451545.) / 36525.
        #correction(precession, nutation)
        sc_fk5 = SkyCoord(ra = ra_2000*u.radian, dec = dec_2000*u.radian, frame = "fk5")
        trans = sc_fk5.transform_to(FK5(equinox = "J{}".format(2000.+tu*100.)))
        ranow = trans.ra.radian
        delnow = trans.dec.radian
        
        x = self.nutation(ranow, delnow)

        #Earth velocity to object
        earthloc = EarthLocation(lon = glongitude*u.deg, lat = glatitude*u.deg, height = gheight*u.m)
        earthloc_gcrs = earthloc.get_gcrs(self.t)
        sc_earth = SkyCoord(earthloc_gcrs).fk5

        v_earth = sc_earth.velocity.d_xyz.value

        vobs = - (v_earth[0] * x_2000[0] + v_earth[1] * x_2000[1] + v_earth[2] * x_2000[2])

        #solar
        sc_fk4_sol = SkyCoord(ra = 18*15*u.deg, dec = 30*u.deg, frame = "fk4")
        trans_sol = sc_fk4_sol.transform_to(FK5(equinox = "J{}".format(2000.+tu*100.)))
        rasol = trans_sol.ra.radian
        delsol = trans_sol.dec.radian
        
        solar_tmp = self.nutation(rasol, delsol)
        v_solar = [solar_tmp[0]*20, solar_tmp[1]*20, solar_tmp[2]*20]
        
        vobs = -(vobs - (v_solar[0] * x[0] + v_solar[1] * x[1] + v_solar[2] * x[2]))


        return vobs

    def nutation(self, ra, dec):
        
        pos = []

        a = math.cos(dec)
        tmp = [a * math.cos(ra), a * math.sin(ra), math.sin(dec)]
        
        #correction(nutation)
        ret = erfa.nut00a(self.t.jd, 0.0)
        nut_long = ret[0]
        nut_obliq = ret[1]
        
        pos.append(tmp[0] - (tmp[1] * math.cos(nut_obliq) + tmp[2] * math.sin(nut_obliq)) * nut_long)
        pos.append(tmp[1] + tmp[0] * math.cos(nut_obliq) * nut_long - tmp[2] * nut_obliq)
        pos.append(tmp[2] + tmp[0] * math.sin(nut_obliq) * nut_long + tmp[1] * nut_obliq)

        return pos

if __name__=="__main__":
    dp = doppler_nanten()
    aa = dp.set_track(83.80613, -5.374320, 0, "j2000", offset_x=0, offset_y=0, offset_dcos=0, offset_coord="j2000", stime=0, restFreq1=230538.000/1000., restFreq2=220398.684/1000., firstsb1=1, firstsb2=-1, secondLO1=8038.000000000/1000., secondLO2=9301.318999999/1000.)
    print("\n\n\n\n\n")
    print(aa)
