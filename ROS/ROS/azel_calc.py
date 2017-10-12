import astropy.units as u
from astropy.coordinates import SkyCoord, EarthLocation, AltAz, get_body
from astropy.time import Time
from datetime import datetime as dt
#import coord
import time
import math
import sys

class azel_calc(object):
    
    latitude = -22.96995611
    longitude = -67.70308139
    height = 4863.85
    #utc_offset = -3. # chile_time
    utc_offset = 0.# nagoya_time

    vel_dt = 0.1
    off_az = 0.
    off_el = 0.
    loop_rate = 0.1

    planet = {0:"sun", 1:"mercury", 2:"venus", 3:"earth", 4:"mars", 5:"jupiter", 6:"saturn", 7:"uranus", 8:"neptune", }

    def dcos_calc(self, x, y, off_x, off_y, dcos):
        if dcos == 1:
            gx = x*math.pi/180.
            gy = y*math.pi/180.
            gy += off_y*math.pi/180
            gx += (off_x*math.pi/180)/math.cos(gy)
            calc_x = gx*180./math.pi
            calc_y = gy*180./math.pi
        else:
            calc_x = x + off_x
            calc_y = y + off_y
        return [calc_x, calc_y]
        



    def velocity_calc(self, az_speed, el_speed, dist, enc_az, enc_el):
        az_list = []
        el_list = []
        move_dist = 0
        tv = time.time()
        n = 0
        while move_dist < dist:
            d_az = (az_speed*self.vel_dt) * n
            d_el = (el_speed*self.vel_dt) * n
            _az = enc_az + d_az
            _el = enc_el + d_el
            az_list.append(_az)
            el_list.append(_el)
            move_dist = abs(d_az) + abs(d_el)
            n += 1
        return [az_list, el_list, tv]

    def coordinate_calc(self, x, y, ntarg, code_mode, off_x, off_y, offcoord, hosei, lamda, dcos, temp, press, humi, now, loop = 500, time_rate=0.):
        print(x, y, ntarg, code_mode, off_x, off_y, offcoord, hosei, lamda, dcos, temp, press, humi, now, loop, time_rate)
        # coordinate check
        if code_mode.lower() == "j2000":
            on_coord = SkyCoord(x, y,frame='fk5', unit='deg',)
        elif code_mode.lower() =="b1950":
            on_coord = SkyCoord(x, y, frame='fk4', unit='deg',)
        elif code_mode.lower() =="galactic":
            on_coord = SkyCoord(x, y, frame='galactic', unit='deg',)
        elif code_mode.lower() =="planet":
            print("planet_move")
        else:
            print("code_mode error !!")
            sys.exit()


        if offcoord.lower() == "j2000":
            off_coord = SkyCoord(off_x,off_y,frame='fk5',unit='arcsec',)
            ret = self.dcos_calc(on_coord.fk5.ra.deg, on_coord.fk5.dec.deg, 
                      off_coord.fk5.ra.deg, off_coord.fk5.dec.deg, dcos)
            real_coord = SkyCoord(ret[0], ret[1], frame='fk5', unit='deg',)

        elif offcoord.lower() == "b1950":
            off_coord = SkyCoord(off_x,off_y,frame='fk4',unit='arcsec',)
            ret = self.dcos_calc(on_coord.fk4.ra.deg, on_coord.fk4.dec.deg, 
                            off_coord.fk4.ra.deg, off_coord.fk4.dec.deg, dcos)
            real_coord = SkyCoord(ret[0], ret[1], frame='fk4', unit='deg',)

        elif offcoord.lower() == "galactic":
            off_coord = SkyCoord(off_x,off_y,frame='galactic',unit='arcsec',)
            ret = self.dcos_calc(on_coord.galactic.l.deg, on_coord.galactic.b.deg, 
                            off_coord.galactic.l.deg, off_coord.galactic.b.deg, dcos)
            real_coord = SkyCoord(ret[0], ret[1], frame='galactic', unit='deg',)

        elif offcoord.lower() == "horizontal":
            self.off_az = off_x #arcsec
            self.off_el = off_y #arcsec
            if not code_mode.lower() =="planet":
                real_coord = on_coord
            else:
                pass
        else:
            print("############## no offcoord ##############")
            pass
        
        # convert_azel
        print("convert")
        az_list= []
        el_list = []
        time_list = []
        print(self.latitude*u.deg)
        print(self.longitude*u.deg)
        nanten2 = EarthLocation(lat = self.latitude*u.deg, lon = self.longitude*u.deg, height = self.height*u.m)
        utcoffset = self.utc_offset*u.hour
        tstr = now.strftime('%Y-%m-%d %H:%M:%S')
        for i in range(loop):
            time_list.append(Time(tstr) - utcoffset + (i*self.loop_rate + time_rate)*u.s)
        atime = Time(time_list)
        if code_mode.lower() =="planet":
            real_coord = get_body(self.planet[ntarg], atime)
            real_coord.location=nanten2
            real_coord.pressure=press*u.Pa#param
            real_coord.temperature=temp*u.deg_C#param
            real_coord.relative_humidity=humi#param
            real_coord.obswl = lamda*u.um#param
            altaz = real_coord.altaz
        else:
            real_coord.location=nanten2
            real_coord.pressure=press*u.Pa#param
            real_coord.temperature=temp*u.deg_C#param
            real_coord.relative_humidity=humi#param
            real_coord.obswl = lamda*u.um#param
            #real_coord.obstime = atime
            altaz = real_coord.transform_to(AltAz(obstime=time_list))

        print("start")
        for i in range(loop):
            ret_azel = self.dcos_calc(altaz[i].az.arcsec, altaz[i].alt.arcsec, self.off_az, self.off_el, dcos)
            _az = ret_azel[0]/3600.*math.pi/180.
            _el = ret_azel[1]/3600.*math.pi/180.
            #ret = coord.apply_kisa(_az, _el, hosei)
            target_az = ret_azel[0]#+ret[0]
            target_el = ret_azel[1]#+ret[1]
            az_list.append(target_az)
            el_list.append(target_el)
            #print(i)
        print("create_list end")
        now = float(now.strftime("%s"))+9*3600. + float(now.strftime("%f"))*1e-6#nagoya
        #now = float(now.strftime("%s"))-3*3600. + float(now.strftime("%f"))*1e-6#chile
        print(az_list[0]/3600.,el_list[0]/3600., now)
        print(now)
        print(len(az_list),len(el_list))
        return[az_list, el_list, now]
            

if __name__ == "__main__":
    qq = azel_calc()
    from datetime import datetime as dt
    now = dt.utcnow()
    qq.coordinate_calc(83, -5, 0, code_mode="j2000", off_x=10, off_y=10, offcoord="j2000", hosei="", lamda=2600, dcos=1, temp=20, press=5, humi=0.07, now=now, loop = 50, time_rate=0.)
    #qq.coordinate_calc(30, 40, 0, code_mode="planet", off_x=10, off_y=10, offcoord="horizontal", hosei="", lamda=2600, dcos=1, temp=20, press=5, humi=0.07, now=now, loop = 50, time_rate=0.)
