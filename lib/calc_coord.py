import astropy.units as u
from astropy.coordinates import SkyCoord, EarthLocation, AltAz, get_body
from astropy.time import Time
from datetime import datetime as dt
from datetime import timedelta
import time
import math
import sys
sys.path.append('/home/amigos/ros/src/necst/lib')
sys.path.append('/home/necst/ros/src/necst/lib')
import coord
import numpy as np
import rospy

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

    coord_list = {"icrs":"icrs", "j2000":"fk5", "fk5":"fk5", "b1950":"fk4", "fk4":"fk4", "gal":"galactic", "galactic":"galactic","altaz":"altaz", "horizontal":"altaz",}
    
    def __init__(self):
        self.coord = coord.coord_calc()
        self.first_time = time.time()
        pass


    def kisa_calc(self, altaz, hosei):
        rospy.loginfo(hosei)
        _az = np.radians(altaz.az.deg)
        _el = np.radians(altaz.alt.deg)
        ret = self.coord.apply_kisa_test(_az, _el, hosei)
        target_az = altaz.az.arcsec+ret[0]
        target_el = altaz.alt.arcsec+ret[1]
        return target_az, target_el

    def kisa_calc2(self, az, el, hosei):
        rospy.loginfo(hosei)
        _az = np.radians(az/3600.)
        _el = np.radians(el/3600.)
        ret = self.coord.apply_kisa_test(_az, _el, hosei)
        target_az = az+ret[0]
        target_el = el+ret[1]
        return target_az, target_el    

    def coordinate_calc(self, x_list, y_list, time_list,  coord, off_az, off_el, hosei, lamda, press, temp, humi, limit=True, rotation=True):
        """
        x_list and y_list == [deg]
        """
        if coord == "planet":
            planet_flag = True
            coord = "altaz"
        else:
            planet_flag = False
            pass
        # coordinate check len(x_list)=len(y_list)=len(time_list)
        frame = self.coord_list[coord.lower()]
        az_list=[]
        el_list=[]    
        if frame == "altaz":
            az_list = list(map(lambda k :k+off_az, x_list))
            el_list = list(map(lambda k :k+off_el, y_list))

            if planet_flag == True:
                tmp_az_list = []
                tmp_el_list = []
                for i in range(len(az_list)):
                    azel_list = self.kisa_calc2(az_list[i],el_list[i], hosei)
                    tmp_az_list.append(azel_list[0]+off_az)
                    tmp_el_list.append(azel_list[1]+off_el)
                az_list = tmp_az_list
                el_list = tmp_el_list
            else:
                pass

        else:
            on_coord = SkyCoord(x_list, y_list,frame=frame, unit='arcsec',)

            # convert_azel
            nanten2 = EarthLocation(lat = self.latitude*u.deg, lon = self.longitude*u.deg, height = self.height*u.m)
            on_coord.location=nanten2
            on_coord.pressure=press*u.hPa #weather correction
            on_coord.temperature=temp*u.deg_C #weather correction
            on_coord.relative_humidity=humi #weather correction
            on_coord.obswl = lamda*u.um #weather correction

            altaz_list = on_coord.transform_to(AltAz(obstime=time_list))

            for i in range(len(altaz_list)):
                azel_list = self.kisa_calc(altaz_list[i], hosei)
                az_list.append(azel_list[0]+off_az)
                el_list.append(azel_list[1]+off_el)
                #az_list.append(altaz_list[i].az.arcsec+off_az) #no_kisa
                #el_list.append(altaz_list[i].alt.arcsec+off_el) #no_kisa

            pass

        if rotation == False:
            az_list = [i-360*3600. if i>180*3600. else i for i in az_list]
            az_list = [i+360*3600. if i<-180*3600. else i for i in az_list]
        #print("az :",az_list[0]/3600.,"el :", el_list[0]/3600.)

        return[az_list, el_list]
            

