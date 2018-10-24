#!/usr/bin/env python3

import time
from datetime import datetime as dt
import sys
import os
import threading

import rospy
from necst.msg import Status_weather_msg
from necst.msg import Status_obs_msg
from necst.msg import String_list_msg
from necst.msg import String_necst

import signal
def handler(signal, frame):
    print("***program end***")    
    sys.exit()
    return
    
signal.signal(signal.SIGINT, handler)

node_name = "obs_log"

obs = ''
obs_flag = False
#observation = ""
weather = ""
hosei = ""
stop = ""
alert = ""
number = 1

def _obs(req):
    global obs
    global start_flag
    obs = req
    start_flag = True
    return

def _weather(req):
    global weather
    weather = req
    return

def _hosei(req):
    global hosei
    hosei = req
    return

def _obs_stop(req):
    global stop
    stop = req.data
    return

def _alert(req):
    global alert
    alert = req.data
    return

def obs_format():
    while not rospy.is_shutdown():
        ctime = dt.utcnow()            
        filename = ctime.strftime("%Y%m%d")
        if os.path.isfile("/home/amigos/data/obs_log/"+filename+".txt"):
            pass
        else:
            time.sleep(10.)
            continue
        f = open("/home/amigos/data/obs_log/"+filename+".txt", "r")
        ff = f.readlines()
        f.close()
        ft = open("/home/amigos/data/log/"+filename+".txt", "a")
        for i in ff:
            if not "@" in i:
                ft.write(i)
            else:
                pass
        ft.close()
        time.sleep(60.)
    return
    
def end():
    if stop:
        try:
            ctime = dt.utcnow()            
            filename = ctime.strftime("%Y%m%d")            
            f = open("/home/amigos/data/obs_log/"+filename+".txt", "w")
            f.write("- limit error"+"\n")
            f.write("-- "+str(stop)+"\n")
            f.write("\n")            
            f.close()
        except:
            pass
        stop = ""
    else:
        pass
    if alert:
        try:
            ctime = dt.utcnow()            
            filename = ctime.strftime("%Y%m%d")
            f = open("/home/amigos/data/obs_log/"+filename+".txt", "r")    
            f.write("- alert message"+"\n")
            f.write("-- "+str(alert)+"\n")
            f.write("\n")
            f.close()
        except:
            pass
        alert = ""         
    else:
        pass
    
    return

def weather_check(weather, mode=""):
    ctime = dt.utcnow()    
    filename = ctime.strftime("%Y%m%d")
    date = ctime.strftime("%Y/%m/%d %H:%M:%S")
    title ='''
#### <i class="fa fa-cloud" aria-hidden="true"></i> Initial Check 
- Weather
- {0} [UTC]
'''
    initial = ''' 
| In Temp [K] | Out Temp [K] | Dome Temp [K] | Cab Temp [K] | In Humi [%] | Out Hum [%] | Wind Dir [deg]  | Wind Sp [m/s] | Pressure [hPa] |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| {1}  | {2} | {3} | {4} | {5} | {6}  | {7} | {8} | {9} |

- weather, radio_meter
    - ASTE Weather monitor
    - APEX Weather monitor
        - Radiometer : xxx [mm]
    - ALMA Weather monitor
    - soratoru-kun
        - xxx

'''
    ctime = dt.utcnow()        
    filename = ctime.strftime("%Y%m%d")
    f = open("/home/amigos/data/obs_log/"+filename+".txt", "a")
    if not mode:
        f.write(title.format(date))
    f.write(initial.format(date,
                           weather.in_temp,
                           weather.out_temp,
                           weather.dome_temp1,
                           weather.cabin_temp1,
                           weather.in_humi,
                           weather.out_humi,
                           weather.wind_dir,
                           weather.wind_sp,
                           weather.press))
    f.write("<!--\n")
    f.write("- hosei_parameter"+"\n")
    if hosei:
        for i in range(0,len(hosei.data)):
            f.write(str(hosei.data[i])+"\n")
    else:
        f.write("no hosei file"+"\n")
    f.write("-->\n")
    f.close()
    print(title.format(date))
    print(initial.format(date,
                         weather.in_temp,
                         weather.out_temp,
                         weather.dome_temp1,
                         weather.cabin_temp1,
                         weather.in_humi,
                         weather.out_humi,
                         weather.wind_dir,
                         weather.wind_sp,
                         weather.press))
    print("save weather")
    time.sleep(1.)
    return

def observer(number=1):
    now = dt.utcnow()
    ctime = now.strftime("%Y/%m/%d %H:%M:%S")
    name='''
### <a name="log{num}" href="#log{num}"><i class="fa fa-cube" aria-hidden="true"></i> Part {num} </a>
- pass message : none
- observer : xxx
    - {utctime} [UTC]
'''
    print(name.format(utctime=ctime, num=number))
    cctime = dt.utcnow()        
    filename = cctime.strftime("%Y%m%d")
    f = open("/home/amigos/data/obs_log/"+filename+".txt", "a")    
    f.write(name.format(utctime=ctime, num=number))
    f.close()
    return

def observation(target="", obs=""):
    now = dt.utcnow()    
    ctime = now.strftime("%Y/%m/%d %H:%M:%S")

    title = '#### <i class="fa fa-stop" aria-hidden="true"></i> {obsmode}'
    if obs.active == True:
        data ='''
- [ UTC ] {obstime} start:
```
{obs_script} 
```
- target : {target} 
- obsfile : {obs_file}
'''
    else:
        data = '''
- [ UTC ] {obstime} end
'''
    cctime = dt.utcnow()            
    filename = cctime.strftime("%Y%m%d")
    f = open("/home/amigos/data/obs_log/"+filename+".txt", "a")
    if target != obs.target:
        print(title.format(obsmode = obs.obsmode))
        f.write(title.format(obsmode = obs.obsmode))
    else:
        pass
    print(data.format(**{"target":obs.target, "obstime":ctime, "obs_script":obs.obs_script, "target":obs.target, "obs_file":obs.obs_file}))
    f.write(data.format(**{"target":obs.target, "obstime":ctime, "obs_script":obs.obs_script, "obs_file":obs.obs_file}))
    if obs.obsmode == "Preparation" or obs.obsmode == "Finalize":
        weather_check(weather,"preparation")
    else:
        pass
    if obs.active == True:
        pass
    else:
        f.write("\n")
    f.close()
    
    
    return obs.target


data = '''        
# <a name="title" href="#title"> NANTEN2 Observation : {day} </a>


<i class="fa fa-link " style="font-size:1em;"></i> NANTEN2 Observation {day}
- [wiki page](http://wiki.a.phys.nagoya-u.ac.jp/5b8f99d28ce0733e00249a27)
- [Google Drive]() 
- [text file](https://drive.google.com/open?id=1_oRGJnRlbkPlg8WVTXmmq8HVYolDEwO3)
- [Alab Wiki NANTEN2 2018 Operation](http://wiki.a.phys.nagoya-u.ac.jp/5acc07388ce0733e0024850d)

## <a name="summary" href="#summary"><i class="fa fa-list-ul " style="font-size:1em;"></i>  Summary </a>

- <a href="#observation">observation menu</a>    
  - <a href="#obs1">observation 1</a> 
  - <a href="#obs2">observation 2</a> 
- <a href="#trouble">trouble</a>
  - <a href="#trouble1">trouble 1</a>
  - <a href="#trouble2">trouble 2</a>
- <a href="#log">log</a>
  - <a href="#log1">  Part1 </a>
  - <a href="#log2"> Part 2 </a>
  - <a href="#log3"> Prrt 3 </a>
  - <a href="#logc"> chile </a>

### <a name="crew" href="#crew"><i class="fa fa-users" aria-hidden="true"></i> Crew </a>

| Part 1 (UTC 00:00~) | Part 2 (UTC 05:00~) | Part 3 (UTC 10:00~) | Part chile (UTC 15:00~) |
| :---: | :---: | :---: | :---: |
| --- | --- | --- | --- |


## <a name="observation" href="#observation"><i class="fa fa-cubes" aria-hidden="true"></i> Observation</a>
### <a name="obs2" href="#obs2"><i class="fa fa-cube" aria-hidden="true"></i> ****** </a>
- observation_time : xx[minute]/1 [obs]

#### purpose
- xxx

#### method
- command
```
xxx
```

#### result
- xxx


## <a name="trouble" href="#trouble"><i class="fa fa-cubes" style="font-size:1em;"></i> Trouble </a>
### <a name="trouble1" href="#trouble1"><i class="fa fa-cube" aria-hidden="true"></i> xxx </a>
- content
    - xxx
- countermeasure
    - xxx


## <a name="log" href="#log"><i class="fa fa-cubes" style="font-size:1em;"></i> LOG </a>
'''

def initialize():
    stime = ""    
    ctime = dt.utcnow()
    day = ctime.strftime("%Y-%m-%d")
    filename = ctime.strftime("%Y%m%d")            
    if ctime.day != stime and not os.path.exists("/home/amigos/data/obs_log/"+filename+".txt"):
        print( os.path.exists("/home/amigos/data/obs_log/"+day+".txt"))
        print(data.format(**{"day":day}))            
        f = open("/home/amigos/data/obs_log/"+filename+".txt", "a")
        f.write(data.format(**{"day":day}))
        f.close()
        weather_check(weather)
        stime = ctime.day
        print("input observe number(1 or 2 or 3)")
    else:
        f = open("/home/amigos/data/obs_log/"+filename+".txt", "a")
        f.write("\n")
        f.close()
        print("input observe number(1 or 2 or 3)")        
        pass

def start_program():
    global weather
    global obs
    global target
    global number
    while not weather:
        print("wait weather data...")
        time.sleep(1.)
    initialize()
    ret = ""
    old_flag = ""
    while not rospy.is_shutdown():
        while not obs:
            time.sleep(1.)
        if  obs.active != old_flag or obs.target != ret:
            ret = observation(ret, obs)
            old_flag = obs.active
            print("input observe number(1 or 2 or 3)")
        else:
            pass
        time.sleep(1.)
    
    return

def observer_change():
    global weather
    time.sleep(3.)
    old_num = ""
    while not rospy.is_shutdown():
        #number = input("input observe number(1 or 2 or 3)")
        number = input("")
        try:
            number = int(number)
        except:
            print("Not integer")
            pass
        if isinstance(number, int) and number != old_num:
            observer(number)
            weather_check(weather)
            old_num = number
        else:
            print("same number")
            pass
    return


if __name__ == "__main__":
    rospy.init_node(node_name)
    sub1 = rospy.Subscriber("obs_status", Status_obs_msg, _obs)
    sub2 = rospy.Subscriber("status_weather", Status_weather_msg, _weather)
    sub3 = rospy.Subscriber("hosei_parameter", String_list_msg, _hosei)
    sub4 = rospy.Subscriber("obs_stop", String_necst, _obs_stop)
    sub5 = rospy.Subscriber("alert", String_necst, _alert, queue_size=1)
    #thread = threading.Thread(target=obs_format)
    #thread.start()
    obs_thread = threading.Thread(target=observer_change)
    obs_thread.start()

    if os.path.exists("/home/amigos/data/obs_log"):
        pass
    else:
        os.mkdir("home/amigos/data/obs_log")
    #if os.path.exists("/home/amigos/data/log"):
        #pass
    #else:
        #os.mkdir("home/amigos/data/log")            
    
    time.sleep(1)
    start_program()

