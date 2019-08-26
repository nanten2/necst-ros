import math
import time
#from pyslalib import slalib
import sys
sys.path.append('/home/necst/ros/src/necst/lib')
import ctypes
#from jplephem.spk import SPK
#import geomech





class coord_calc(object):
    
    eqrau = [0.0,
        2440.0,            # Mercury
        6051.8,            # Venus
        0.0,
        3389.9,            # Mars
        69134.1,           # Jupiter
        57239.9,           # Saturn
        25264.3,           # Uranus
        24553.1,           # Neptune
        1151.0,            # Pluto
        1737.53,           # Moon
        696000.0          ]# Sun
    tai_utc = 36.0 # tai_utc=TAI-UTC  2015 July from ftp://maia.usno.navy.mil/ser7/tai-utc.dat
    
    
    def __init__(self):
        #self.jpl = SPK.open('./de430.bsp')
        # from https://pypi.python.org/pypi/jplephem
        #self.geomech = geomech.geomech_monitor_client('172.20.0.12',8101)
        pass
    '''
    def calc_jd_utc(self):
        h = time.gmtime()
        ret = slalib.sla_caldj(h.tm_year, h.tm_mon, h.tm_mday) # ret[0] = MJD
        jd_utc = ret[0]+2400000.5+h.tm_hour/24.0+h.tm_min/1440.0+h.tm_sec/86400.0
        return jd_utc
    
    def calc_planet_coordJ2000(self, ntarg):
        err_code = i = nctr = 3
        c = 2.99792e8
        k_to_au = 6.68459e-9
        
        jd_utc = self.calc_jd_utc()
        jd = jd_utc + (self.tai_utc + 32.184) / (24. * 3600.)  # Convert UTC to Dynamical Time
        if ntarg != 10:
            if ntarg == 11: #for Sun
                n_ntarg = 10
            else:
                n_ntarg = ntarg
            position = self.jpl[0, n_ntarg].compute(jd) # compute planet Barycenter
            position -= self.jpl[0, 3].compute(jd) # compute Earth Barycenter
            position -= self.jpl[3, 399].compute(jd) # compute Earth position
            dist = math.sqrt(position[0] ** 2 + position[1] ** 2 + position[2] ** 2) # [km]
            
            position_1 = self.jpl[0, 3].compute(jd) #Earth position at jd
            position_2 = self.jpl[0, n_ntarg].compute(jd - (dist / c) / (24. * 3600.)) # Target position when the light left
            position = position_2 - position_1
            position -= self.jpl[3, 399].compute(jd)
            dist = math.sqrt(position[0] ** 2 + position[1] ** 2 + position[2] ** 2)
        else: #for Moon
            position = self.jpl[3, 301].compute(jd)
            dist = math.sqrt(position[0] ** 2 + position[1] ** 2 + position[2] ** 2) # [km]
            
            position = self.jpl[3,301].compute(jd - (dist / c) / (24 * 3600.0))
            dist = math.sqrt(position[0] ** 2 + position[1] ** 2 + position[2] ** 2)
        ret = slalib.sla_dcc2s(position)
        ra = ret[0] # radian
        dec = ret[1] # radian
        ra = slalib.sla_dranrm(ra)
        radi = math.asin(self.eqrau[ntarg] / dist)
        dist = dist*k_to_au
        return [ra, dec, dist, radi]
    
    def planet_J2000_geo_to_topo(self, gra, gdec, dist, radi, dut1, longitude, latitude, height):
        jd_utc = self.calc_jd_utc()
        date = jd_utc - 2400000.5 + dut1 / (24. * 3600.)
        jd = jd_utc - 2400000.5 + (self.tai_utc + 32.184) / (24. * 3600.) # reference => http://www.cv.nrao.edu/~rfisher/Ephemerides/times.html
        
        # Spherical to x,y,z 
        v = slalib.sla_dcs2c(gra, gdec)
        for i in range (3):
            v[i] *= dist
        
        # Precession to date. 
        rmat = slalib.sla_prec(2000.0, slalib.sla_epj(jd))
        vgp = slalib.sla_dmxv(rmat, v)
        
        # Geocenter to observer (date). 
        stl = slalib.sla_gmst(date) + longitude
        vgo = slalib.sla_pvobs(latitude, height, stl)
        
        # Observer to planet (date). 
        for i in range (3):
            v[i] = vgp[i] - vgo[i]
        
        disttmp = dist
        dist = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        radi *= disttmp / dist
        
        # Precession to J2000 
        rmat = slalib.sla_prec(slalib.sla_epj(jd), 2000.)
        vgp = slalib.sla_dmxv(rmat, v)
        
        # To RA,Dec. 
        ret = slalib.sla_dcc2s(vgp)
        tra = slalib.sla_dranrm(ret[0])
        tdec = ret[1]
        return [dist, radi, tra, tdec]
        '''
    
    def apply_kisa(self, az, el, hosei):
        """ from [coordinate.cpp]
        #kisa parameter
        (daz[0], de[1], kai_az[2], omega_az[3], eps[4], kai2_az[5], omega2_az[6], kai_el[7], omega_el[8], kai2_el[9], omega2_el[10], g[11], gg[12], ggg[13], gggg[14],
        del[15], de_radio[16], del_radio[17], cor_v[18], cor_p[19], g_radio[20], gg_radio[21], ggg_radio[22], gggg_radio[23])
        """
        kisa = self.read_kisa_file(hosei,24)
        #geo_kisa = self.read_kisa_file("hosei_opt_geomech.txt",10)
        """
        #geo_kisa parameter
        (kai[0], omega[1], gxc[2], gyc[3], scx[4], scy[5], tx1[6], tx2[7], ty1[8], ty2[9])
        """
        DEG2RAD = math.pi/180
        RAD2DEG = 180/math.pi
        ARCSEC2RAD = math.pi/(180*60*60.)
        kisa[3] = kisa[3]*DEG2RAD
        kisa[6] = kisa[6]*DEG2RAD
        kisa[8] = kisa[8]*DEG2RAD
        kisa[10] = kisa[10]*DEG2RAD
        kisa[19] = kisa[19]*DEG2RAD
        el_d = el*RAD2DEG
        delta = [0,0]

        # reference from src/coord/correct.h
        # line 242, 248
        # (*) : not using now.

        """ dx model.
        l1 : Az axis inclination
        l2 : Az-El skew sngle
        l3 : Az encoder offset
        l4 : Az offset
        l5 : 180 deg. periodic term
        l6 : Az offset (radio)
        l7 : Collimation error (radio)
        """

        dx = kisa[2] * math.sin(kisa[3] - az) * math.sin(el) \
          + kisa[4] * math.sin(el) \
          + kisa[0] * math.cos(el) \
          + kisa[1] \
          + kisa[5] * math.cos(2 * (kisa[6] - az)) * math.sin(el) \
          + kisa[16] \
          + kisa[18] * math.cos(el + kisa[19])

        delta[0] = -dx # arcsec.

        """ dy model.
        l1 : Az axis inclination
        l2 : 180 deg. periodic term
        l3 : El offset
        l4 : Optical gravitational deflection
        l5 : Optical gravitational deflection (*)
        l6 : Optical gravitational deflection (*)
        l7 : Optical gravitational deflection (*)
        l8 : El offset (radio)
        l9 : Collimation error (radio)
        l10 : Radio gravitational deflection
        l11 : Radio gravitational deflection
        l12 : Radio gravitational deflection
        l13 :Radio gravitational deflection
        """

        dy = - kisa[2] * math.cos(kisa[3] - az) \
          - kisa[5] * math.sin(2 * (kisa[6] - az)) \
          + kisa[15] \
          + kisa[11] * el_d \
          + kisa[12] * el_d * el_d \
          + kisa[13] * el_d * el_d * el_d \
          + kisa[14] * el_d * el_d * el_d * el_d \
          + kisa[17] \
          - kisa[18] * math.sin(el + kisa[19]) \
          + kisa[20] * el_d \
          + kisa[21] * el_d * el_d \
          + kisa[22] * el_d * el_d * el_d \
          + kisa[23] * el_d * el_d * el_d * el_d

        delta[1] = -dy # arcsec.

        if(math.fabs(math.cos(el))>0.001):
            delta[0]=delta[0]/math.cos(el)

        return delta

    def apply_kisa_test(self, az, el, hosei):
        """ from [coordinate.cpp]
        #kisa parameter
        (daz[0], de[1], kai_az[2], omega_az[3], eps[4], kai2_az[5], omega2_az[6], kai_el[7], omega_el[8], kai2_el[9], omega2_el[10], g[11], gg[12], ggg[13], gggg[14],
        del[15], de_radio[16], del_radio[17], cor_v[18], cor_p[19], g_radio[20], gg_radio[21], ggg_radio[22], gggg_radio[23])
        """
        kisa = self.read_kisa_file(hosei,24)
        
        DEG2RAD = math.pi/180
        RAD2DEG = 180/math.pi
        ARCSEC2RAD = math.pi/(180*60*60.)
        kisa[3] = kisa[3]*DEG2RAD
        kisa[6] = kisa[6]*DEG2RAD
        kisa[8] = kisa[8]*DEG2RAD
        kisa[10] = kisa[10]*DEG2RAD
        kisa[19] = kisa[19]*DEG2RAD
        el_d = el*RAD2DEG
        delta = [0,0]
        
        # reference from src/coord/correct.h 
        # line 242, 248
        dx = kisa[2]*math.sin(kisa[3]-az)*math.sin(el)+kisa[4]*math.sin(el)+kisa[0]*math.cos(el)+kisa[1]+kisa[5]*math.sin(2*(kisa[6]-az))*math.sin(el)\
            +kisa[16]+kisa[18]*math.cos(el+kisa[19])
        delta[0] = -dx # arcsec
        
        dy = -kisa[7]*math.cos(kisa[8]-az)-kisa[9]*math.cos(2*(kisa[10]-az))+kisa[15]+kisa[11]*el_d+kisa[12]*el_d*el_d+kisa[13]*el_d*el_d*el_d+kisa[14]*el_d*el_d*el_d*el_d\
            +kisa[17]-kisa[18]*math.sin(el+kisa[19])+kisa[20]*el_d+kisa[21]*el_d*el_d+kisa[22]*el_d*el_d*el_d+kisa[23]*el_d*el_d*el_d*el_d
        delta[1] = -dy # arcsec
        if(math.fabs(math.cos(el))>0.001):
            delta[0]=delta[0]/math.cos(el)
        
        return delta


    def read_kisa_file(self, hosei, num):
        try:
            f = open('/home/necst/ros/src/necst/lib/' + hosei)
        except:
            f = open('/home/amigos/ros/src/necst/lib/' + hosei)
        line = f.readline()
        kisa = [0]*num
        n = 0

        while line:
            line = line.rstrip()
            kisa[n] = float(line)
            line = f.readline()
            n = n+1
        f.close

        #apply hosei file
        """
        f = open(diff_f)
        line = f.readline()
        diff = [0]*24
        n = 0
        while line:
            line = line.rstrip()
            diff[n] = line
            line = f.readline()
            n = n+1
        f.close()
        kisa = [kisa[i]+diff[i] for i in range(kisa)]
        """
        return kisa
