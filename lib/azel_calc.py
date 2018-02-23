import astropy.units as u
from astropy.coordinates import SkyCoord, EarthLocation, AltAz, get_body
from astropy.time import Time
from datetime import datetime as dt
import time
import math
import sys
sys.path.append('/home/amigos/ros/src/necst/lib')
sys.path.append('/home/necst/ros/src/necst/lib')
import coord
import numpy as np

class azel_calc(object):
    
    latitude = -22.96995611
    longitude = -67.70308139
    height = 4863.85
    utc_offset = 0.# utc_time
    soft_limit = 240.

    vel_dt = 0.1
    off_az = 0.
    off_el = 0.
    loop_rate = 0.1

    planet = {1:"mercury", 2:"venus", 3:"earth", 4:"mars", 5:"jupiter", 6:"saturn", 7:"uranus", 8:"neptune", 10:"moon", 11:"sun"}

    def __init__(self):
        self.coord = coord.coord_calc()
        pass


    def dcos_calc(self, x, y, off_x, off_y, dcos):
        if dcos == 1:
            y += off_y
            x += off_x/np.cos(np.radians(y))            
        else:
            y += off_y
            x += off_x
        return [x, y]
        
    def kisa_calc(self, altaz, dcos, hosei):
        ret_azel = self.dcos_calc(altaz.az.arcsec, altaz.alt.arcsec, self.off_az, self.off_el, dcos)
        #ret_azel = self.dcos_calc(40, 50, self.off_az, self.off_el, dcos)
        #ret_azel = [60,30]
        #print(str(num))
        _az = np.radians(ret_azel[0]/3600.)
        _el = np.radians(ret_azel[1]/3600.)
        ret = self.coord.apply_kisa_test(_az, _el, hosei)
        target_az = ret_azel[0]+ret[0]
        target_el = ret_azel[1]+ret[1]
        return target_az, target_el

    def azel_calc(self, az, el, off_x, off_y, off_coord, now, vel_x=0, vel_y=0, movetime=10):
        if off_coord.lower() != "horizontal":
            print("Please, off_coord is HORIZONTAL")
            return
        else:
            pass
        tv = time.time()
        az_list = [(az+off_x)*3600.+vel_x*0.1*i for i in range(int(movetime*10))]
        el_list = [(el+off_y)*3600.+vel_y*0.1*i for i in range(int(movetime*10))]
        return [az_list, el_list, tv]

    def coordinate_calc(self, x, y, coord, ntarg, off_x, off_y, offcoord, hosei, lamda, dcos, temp, press, humi, now, movetime = 10):
        print("parameter : ", x, y, ntarg, coord, off_x, off_y, offcoord, hosei, lamda, dcos, temp, press, humi, now, movetime)
        print("site position(latitude,longitude) : ", (self.latitude*u.deg, self.longitude*u.deg))
        # coordinate check
        if coord.lower() == "j2000":
            on_coord = SkyCoord(x, y,frame='fk5', unit='deg',)
        elif coord.lower() =="b1950":
            on_coord = SkyCoord(x, y, frame='fk4', unit='deg',)
        elif coord.lower() =="galactic":
            on_coord = SkyCoord(x, y, frame='galactic', unit='deg',)
        elif coord.lower() =="planet":
            print("planet_move")
        else:
            print("coord error !!")
            sys.exit()


        if offcoord.lower() == "j2000" or offcoord.lower() == "equatorial":
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
            if not coord.lower() =="planet":
                real_coord = on_coord
            else:
                pass
        else:
            print("############## no offcoord ##############")
            pass
        
        # convert_azel
        nanten2 = EarthLocation(lat = self.latitude*u.deg, lon = self.longitude*u.deg, height = self.height*u.m)
        #tstr = now.strftime('%Y-%m-%d %H:%M:%S')
        time_list = [Time(now)-self.utc_offset*u.hour+(i*self.loop_rate)*u.s for i in range(int(movetime*10))]
        if coord.lower() =="planet":
            real_coord = get_body(self.planet[ntarg], Time(now))
        else:
            pass
        real_coord.location=nanten2
        real_coord.pressure=press*u.Pa#param
        real_coord.temperature=temp*u.deg_C#param
        real_coord.relative_humidity=humi#param
        real_coord.obswl = lamda*u.um#param
        altaz = real_coord.transform_to(AltAz(obstime=time_list))

        if int(movetime*100) == 1:
            list_num = int(len(x))
            print("###otf_mode###")
        else:
            list_num = int(movetime*10)
        print("create_list : start!!")
        altaz_list = [altaz[i] for i in range(list_num)]# shorting calc time
        #az_list = [self.kisa_calc(altaz_list[i], dcos, hosei)[0] for i in range(int(movetime*10))]
        #el_list = [self.kisa_calc(altaz_list[i], dcos, hosei)[1] for i in range(int(movetime*10))]
        az_list=[]
        el_list=[]
        for i in range(list_num):
            azel_list = self.kisa_calc(altaz_list[i], dcos, hosei)
            az_list.append(azel_list[0])
            el_list.append(azel_list[1])
        check_list1 = [i for i in az_list if 0 <= i <= self.soft_limit*3600. ]
        check_list2 = [i for i in az_list if (360-self.soft_limit)*3600. <= i <= 360*3600. ]
        
        if check_list1 == az_list:
            pass
        elif check_list2 == az_list:
            az_list = [i-360.*3600. for i in az_list]
        else:
            check_list3 = [i for i in az_list if 0 <= i < 270.*3600. ]
            check_list4 = [i for i in az_list if 90*3600. < i <= 360.*3600. ]
            if check_list3 == az_list:
                pass
            elif check_list4 == az_list:
                az_list = [i-360.*3600. for i in az_list]
            else:
                pass
        
    
                
        print("create_list : end!!")

        now = float(now.strftime("%s")) + float(now.strftime("%f"))*1e-6#utc
        print("az :",az_list[0]/3600.,"el :", el_list[0]/3600., "time : ", now)
        return[az_list, el_list, now]
            

if __name__ == "__main__":
    qq = azel_calc()
    from datetime import datetime as dt
    now = dt.utcnow()
    qq.coordinate_calc([30,23,23], [40,23,34], "j2000", 7, off_x=10, off_y=10, offcoord="horizontal", hosei="hosei_230.txt", lamda=2600, dcos=1, temp=20, press=5, humi=0.07, now=now, movetime = 0.1)
    
    
