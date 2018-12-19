import time
from datetime import datetime as dt
from astropy.coordinates import SkyCoord, AltAz, FK5, EarthLocation
import astropy.units as u
from astropy.time import Time
from datetime import datetime as dt
import time
from datetime import timedelta
nanten2 = EarthLocation(lat = -22.96995611*u.deg, lon = -67.70308139*u.deg, height = 4863.85*u.m)
from firebase import firebase


def initialize():
    global fb
    url, passwd, mail = read_conf()
    fb = firebase.FirebaseApplication(url,None )
    #f = open("pass.txt")
    auth = firebase.FirebaseAuthentication(passwd, mail, extra={"id":123})
    fb.authentication = auth
    return
    
def read_conf():
    f = open("/home/amigos/tmp/pass.txt", "r")
    ff = f.readlines()
    ff = ff[0].split()
    f.close()
    return ff[0], ff[1], ff[2]
    
    
def calc_obstime(now, obsfile):
    global date
    el_limit = 20
    with open("/home/amigos/necst-obsfiles/"+obsfile, "r") as f:
        obs = f.readlines()
    for i in obs:
        if i.split()[0].split("=")[0] == "lambda_on":
            x = i.split()[0].split("=")[1]
            print(x)
        elif i.split()[0].split("=")[0] == "beta_on":
            y = i.split()[0].split("=")[1]
            print(y)
            break
        else:
            pass
    coord = SkyCoord(x, y, unit="deg", frame="fk5", location=nanten2)
    #now = dt.utcnow()
    date = now.strftime("%Y/%m/%d")
    dtime = now.strftime("%H:%M:%S")
    timestamp = dt.timestamp(now)
    time_list = [now+timedelta(minutes=i) for i in range(60*24)]# 1days
    timestamp_list = [timestamp + i*60 for i in range(60*24)]
    calc_coord = coord.transform_to(AltAz(obstime=Time(time_list)))
    #for i in calc:
    #    print(i.alt.deg)
    rising_time = [timestamp_list[i] if calc_coord[i].alt.deg >20 else 0 for i in range(len(calc_coord.alt.deg))]
    start = []
    end = []
    for i in range(len(calc_coord.alt.deg)):
        if len(start) == len(end):
            if calc_coord[i].alt.deg > el_limit:
                start.append(timestamp_list[i])
            else:
                continue
        if len(start) != len(end):
            if calc_coord[i].alt.deg < el_limit:
                end.append(timestamp_list[i-1])
            else:
                continue
    if len(start) == len(end):
        start.append(0)
        end.append(0)
    elif len(start)>len(end):
        end.append("1E10")
    else:
        pass
    print(start)
    print(end)
    #data = ["0x2", date+" "+dtime, start , end, "ROS_cross_point.py", obsfile, 20, 30]
    data = [2, date+" "+dtime, start , end, "ROS_cross_point.py", obsfile, 20, 30]
    return data

def insert(data):
    if isinstance(data, list):
        fb.put("","queue_list/"+date+"/", data)
    else:
        fb.put("","queue_list/"+date+"/"+str(count), data)
    #count += 1
    return

if __name__=="__main__":
    print("initialize")
    initialize()
    now = dt.strptime("2018/12/19 0:0:0", "%Y/%m/%d %H:%M:%S")
    print("create list")
    obs_list = ["line_cross_m17sw.obs", "line_cross_OriKL_test.obs","line_cross_IRC10216.obs"]
    data = [calc_obstime(now, i) for i in obs_list]
    print("insert data...")
    insert(data)
    print("end")

