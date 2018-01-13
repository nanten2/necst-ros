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
        geo_kisa = self.read_kisa_file("hosei_opt_geomech.txt",10)
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
        dx = kisa[2]*math.sin(kisa[3]-az)*math.sin(el)+kisa[4]*math.sin(el)+kisa[0]*math.cos(el)+kisa[1]+kisa[5]*math.cos(2*(kisa[3]-az))*math.sin(el)\
            +kisa[16]+kisa[18]*math.cos(el+kisa[19])
        delta[0] = -dx # arcsec
        
        dy = -kisa[7]*math.cos(kisa[8]-az)-kisa[9]*math.sin(2*(kisa[10]-az))+kisa[15]+kisa[11]*el_d+kisa[12]*el_d*el_d+kisa[13]*el_d*el_d*el_d+kisa[14]*el_d*el_d*el_d*el_d\
            +kisa[17]-kisa[18]*math.sin(el+kisa[19])+kisa[20]*el_d+kisa[21]*el_d*el_d+kisa[22]*el_d*el_d*el_d+kisa[23]*el_d*el_d*el_d*el_d
        delta[1] = -dy # arcsec
        if(math.fabs(math.cos(el))>0.001):
            delta[0]=delta[0]/math.cos(el)
        
        geo_x = geo_kisa[0]*(-math.sin((geo_kisa[1]-az)*(math.pi/180.))+math.cos((geo_kisa[1]-az)*(math.pi/180.)))+geo_kisa[2]
        geo_y = geo_kisa[0]*(-math.sin((geo_kisa[1]-az)*(math.pi/180.))-math.cos((geo_kisa[1]-az)*(math.pi/180.)))+geo_kisa[3]
        ret = self.geomech.read_geomech_col() # ret[0] = geomech_x, ret[1] = geomech_y
        
        gx = ret[0]-geo_x
        gy = ret[1]-geo_y
        ggx = -((gx+gy)/math.sqrt(2))*math.sin(el*(math.pi/180.)) # arcsec
        ggy = -(gx-gy)/math.sqrt(2) # arcsec
        
        delta[0] = delta[0]-ggx
        delta[1] = delta[1]-ggy
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
        dx = kisa[2]*math.sin(kisa[3]-az)*math.sin(el)+kisa[4]*math.sin(el)+kisa[0]*math.cos(el)+kisa[1]+kisa[5]*math.cos(2*(kisa[3]-az))*math.sin(el)\
            +kisa[16]+kisa[18]*math.cos(el+kisa[19])
        delta[0] = -dx # arcsec
        
        dy = -kisa[7]*math.cos(kisa[8]-az)-kisa[9]*math.sin(2*(kisa[10]-az))+kisa[15]+kisa[11]*el_d+kisa[12]*el_d*el_d+kisa[13]*el_d*el_d*el_d+kisa[14]*el_d*el_d*el_d*el_d\
            +kisa[17]-kisa[18]*math.sin(el+kisa[19])+kisa[20]*el_d+kisa[21]*el_d*el_d+kisa[22]*el_d*el_d*el_d+kisa[23]*el_d*el_d*el_d*el_d
        delta[1] = -dy # arcsec
        if(math.fabs(math.cos(el))>0.001):
            delta[0]=delta[0]/math.cos(el)
        
        return delta
    
    
    '''
    def calc_vobs_fk5(self, ra_2000, dec_2000, gcalc_flag):
        x_2000 = x = x1 = v = v_rev = v_rot = v2 = solx = solv = solx1 =[0,0,0]
        jd_utc = self.calc_jd_utc()
        jd = jd_utc + (self.tai_utc + 32.184) / (24. * 3600.)
        
        #ra_2000=DEG2RAD
        #dec_2000=DEG2RAD
        a = math.cos(dec_2000)
        x_2000[0] = a*math.cos(ra_2000)
        x_2000[1] = a*math.sin(ra_2000)
        x_2000[2]= math.sin(dec_2000)
        
        tu= (jd - 2451545.)/36525.
        ret = slalib.sla_preces( "FK5", 2000., 2000.+tu*100., ra_2000, dec_2000)
        #ret[0] =ranow,    ret[1] = delow
        
        a = math.cos(ret[1])
        x[0] = a*math.cos(ret[0])
        x[1] = a*math.sin(ret[0])
        x[2] = math.sin(ret[1])
        
        ret = slalib.sla_nutc(jd-2400000.5)
        #ret[0] = nut_long, ret[1] = nut_obliq, ret[2] = eps0
        nut_long = ret[0]
        nut_obliq = ret[1]
        eps0 = ret[2]
        x1[0]=x[0]-(x[1]*math.cos(ret[2])+x[2]*math.sin(ret[2]))*ret[0]
        x1[1]=x[1]+x[0]*math.cos(ret[2])*ret[0]-x[2]*ret[1]
        x1[2]=x[2]+x[0]*math.sin(ret[2])*ret[0]+x[1]*ret[1]
        
        x[0]=x1[0]
        x[1]=x1[1]
        x[2]=x1[2]
        v0= 47.404704e-3
        
        ramda=35999.3729*tu+100.4664+(1.9146-0.0048*tu)*math.cos((35999.05*tu+267.53)*DEG2RAD)+0.0200*math.cos((71998.1*tu+265.1)*DEG2RAD)
        
        r=1.000141+(0.016707-0.000042*tu)*math.cos((35999.05*tu+177.53)*DEG2RAD)+0.000140*math.cos((71998.*tu+175.)*DEG2RAD)
        
        ramda1=628.308+(20.995-0.053*tu)*math.cos((35999.5*tu+357.52)*DEG2RAD)+0.439*math.cos((71998.1*tu+355.1)*DEG2RAD)\
            +0.243*math.cos((445267.*tu+298.)*DEG2RAD)
        
        beta=0.024*math.cos((483202.*tu+273.)*DEG2RAD)
        
        r1 = (10.497-0.026*tu)*math.cos((35999.05*tu+267.53)*DEG2RAD)+0.243*math.cos((445267.*tu+28.)*DEG2RAD)+0.176*math.cos((71998.*tu+265.)*DEG2RAD)
        
        ramda   = ramda *DEG2RAD
        
        v[0] = -r*ramda1*math.sin(ramda)+r1*math.cos(ramda)
        v[1] = r*ramda1*math.cos(ramda)+r1*math.sin(ramda)
        v[2] = r*beta
        
        v[0] = v[0]-(0.263*math.cos((3034.9*tu+124.4)*DEG2RAD)+0.058*math.cos((1222. *tu+140.)*DEG2RAD)+0.013*math.cos((6069. *tu+144.)*DEG2RAD))
        v[1] = v[1]-(0.263*math.cos((3034.9*tu+34.4)*DEG2RAD)+0.058*math.cos((1222. *tu+50.)*DEG2RAD)+0.013*math.cos((6069. *tu+54.)*DEG2RAD))
            
        v[0] = v[0]*v0
        v[1] = v[1]*v0
        v[2] = v[2]*v0
        
        e = (23.439291-0.013004*tu)*3600.
        
        v_rev[0] = v[0]
        v_rev[1] = v[1]*math.cos(e*ARCSEC2RAD)-v[2]*math.sin(e*ARCSEC2RAD)
        v_rev[2] = v[1]*math.sin(e*ARCSEC2RAD)+v[2]*math.cos(e*ARCSEC2RAD)
        
        v_e = (465.1e-3)*(1.+0.0001568*gheight/1000.)*math.cos(glatitude)/math.sqrt(1.+0.0066945*math.pow(math.sin(glatitude),2.0))
        
        am = 18.*3600.+41.*60.+50.54841+8640184.812866*tu+0.093104*tu*tu-0.0000062*tu*tu*tu
        
        gmst = (jd-0.5-(long)(jd-0.5))*24.*3600.+am-12.*3600.
        
        l = 280.4664*3600.+129602771.36*tu- 1.093*tu*tu
        l = l*ARCSEC2RAD
        p = (282.937+1.720*tu)*3600.
        p = p*ARCSEC2RAD
        
        w = (125.045-1934.136*tu+0.002*tu*tu)*3600.
        w = w*ARCSEC2RAD
        ll = (218.317+481267.881*tu-0.001*tu*tu)*3600.
        ll = ll*ARCSEC2RAD
        pp = (83.353+4069.014*tu-0.010*tu*tu)*3600.
        pp = pp*ARCSEC2RAD
        dpsi = (-17.1996-0.01742*tu)*math.sin(w)+(-1.3187)*math.sin(2*l)+0.2062*math.sin(2*w)+0.1426*math.sin(l-p)-0.0517*math.sin(3*l-p)+0.0217*math.sin(l+p)\
                +0.0129*math.sin(2*l-w)-0.2274*math.sin(2*ll)+0.0712*math.sin(ll-pp)-0.0386*math.sin(2*ll-w)-0.0301*math.sin(3*ll-pp)\
                -0.0158*sin(-ll+3*l-pp)+0.0123*sin(ll+pp)
        e = e*ARCSEC2RAD
        
        dpsicose = dpsi*math.cos(e)
        
        lst = gmst+(dpsicose+glongitude*RAD2DEG*3600.)/15.
        
        v_rot[0] = -v_e*math.sin(lst*SEC2RAD)
        v_rot[1] = v_e*math.cos(lst*SEC2RAD)
        v_rot[2] = 0.
        
        v2[0] = v_rev[0]+v_rot[0]
        v2[1] = v_rev[1]+v_rot[1]
        v2[2] = v_rev[2]+v_rot[2]
        
        vobs = -(v2[0]*x_2000[0]+v2[1]*x_2000[1]+v2[2]*x_2000[2])
        rasol = 18.*15.*DEG2RAD
        delsol = 30.*DEG2RAD
            
        #slaPreces( "FK4", 1950.,2000.+tu*100.,&rasol,&delsol)
        ret = slalib.sla_preces( "FK4", 1900.,2000.+tu*100.,rasol,delsol)
        #ret[0]=rasol, ret[1]=delsol
        a = math.cos(ret[1])
        solx[0] = a*math.cos(ret[0])
        solx[1] = a*math.sin(ret[0])
        solx[2] = math.sin(ret[1])
        
        """
        solx1[0] = solx[0] - (solx[1] * cos(nut_obliq) + solx[2] * \
            sin(nut_obliq)) * nut_long;
        solx1[1] = solx[1] + (solx[0] * cos(nut_obliq) * nut_long\
            - solx[2] * nut_obliq);
        solx1[2] = solx[2] + (solx[0] * sin(nut_obliq) * nut_long \
            + solx[1] * nut_obliq);
        """
        solx1[0] = solx[0]-(solx[1]*math.cos(eps0)+solx[2]*math.sin(eps0))* nut_long
        solx1[1] = solx[1]+(solx[0]*math.cos(eps0)*nut_long-solx[2]*nut_obliq)
        solx1[2] = solx[2]+(solx[0]*math.sin(eps0)*nut_long+solx[1]*nut_obliq)
        
        solv[0]=solx1[0]*20.
        solv[1]=solx1[1]*20.
        solv[2]=solx1[2]*20.
        
        vobs = vobs-(solv[0]*x[0]+solv[1]*x[1]+solv[2]*x[2])
        vobs = -vobs
        
        #printf("vobs=%f\n",vobs);
        if gcalc_flag == 1:
            return vobs
        elif gcalc_flag == 2:
            return lst
            '''

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
