#! /usr/bin/env python

import time
import os.path
import urllib.request


# Info
# ----

"""
name = 'obs_log'
description = 'Logging observation'
"""

# paremeter
# ----

tau = ''
list = ''

# Main
# ====

def day_start():
    daystmp = time.strftime("%Y%m%d")
    if os.path.isfile("/tmp/"+daystmp+".txt") == False:
    #if os.path.isfile("/home/amigos/NECST/script/data/obs_log/"+daystmp+".txt") == False:
        print('in')
        f = open("/tmp/"+daystmp+".txt", "a")
        f.write("*** JST "+time.strftime("%y/%m/%d")+" ***\n")
        f.close()
    print('out')

def start_script(name, list = ""):
    day_start()
    ut = time.gmtime()
    tstmp = time.strftime("%H:%M:%S", ut)
    daystmp = time.strftime("%Y%m%d")
    #f = open("/home/amigos/NECST/script/data/obs_log/"+daystmp+".txt", "a")
    f = open("/tmp/"+daystmp+".txt", "a")
    if name == "initialize":
        f.write("\n*** Preperation\n")
    elif name == "finalize":
        f.write("\n*** Finalize\n")
    else:
        pass
    f.write("- [UTC] "+tstmp+" start : \n")
    arg = " "+name+" "
    num = len(list)
    if num != 0:
        for i in range(int(num/2)):
            arg += str(list[i*2])+" "+str(list[i*2+1])+" "
    f.write(arg+"\n")
    f.close()

def end_script(name, file = '', start_tsys_12 = '',start_tsys_13 = '', end_tsys_12 = '', end_tsys_13 = '', tau = ''):
    day_start()
    ut = time.gmtime()
    tstmp = time.strftime("%H:%M:%S", ut)
    daystmp = time.strftime("%Y%m%d")
    #f = open("/home/amigos/NECST/script/data/obs_log/"+daystmp+".txt", "a")
    f = open("/tmp/"+daystmp+".txt", "a")
    f.write("-- "+tstmp+" end : "+name+"\n")
    if file:
        f.write("-- datadir : "+file+"\n")
    if start_tsys_12 and start_tsys_13:
        f.write("-- start_tsys : "+start_tsys_12+" K(12CO), "+start_tsys_13+" K(13CO)"+"\n")
    elif start_tsys_12:
        f.write("-- start_tsys : "+start_tsys_12+" K(12CO)"+"\n")
    elif start_tsys_13:
        f.write("-- start_tsys : "+start_tsys_13+" K(13CO)"+"\n")
    if end_tsys_12 and end_tsys_13:
        f.write("-- end_tsys : "+end_tsys_12+" K(12CO), "+end_tsys_13+" K(13CO)"+"\n")
    elif end_tsys_12:
        f.write("-- end_tsys : "+end_tsys_12+" K(12CO)"+"\n")
    elif end_tsys_13:
        f.write("-- end_tsys : "+end_tsys_13+" K(13CO)"+"\n")
    if tau:
        f.write("-- tau : "+tau+"\n")
    if name == "initialize":
        f.write("\n*** Observation\n")
    print(tau)
    f.close()

def weather_log():
    ut = time.gmtime()
    tstmp = time.strftime("%Y/%m/%d %H:%M:%S", ut)
    daystmp = time.strftime("%Y%m%d")
    text = []
    fp = urllib.request.urlopen("http://200.91.8.66/WeatherMonitor/WeatherMenu.html")
    html = fp.readline()
    while html:
        html = html.decode('utf-8')
        html.replace(">", " ")
        text.append(html)
        html = fp.readline()
    fp.close()
    
    in_temp = text[23].split()
    out_temp = text[27].split(">")
    out_temp = out_temp[1].split()
    d_temp = text[31].split()
    c_temp = text[35].split()
    in_humi = text[39].split(">")
    in_humi = in_humi[1].split()
    out_humi = text[43].split(">")
    out_humi = out_humi[1].split()
    wind_dir = text[47].split(">")
    wind_dir = wind_dir[1].split()
    wind_speed = text[51].split()
    press = text[55].split(">")
    press = press[1].split()
    #radiometer

    #f = open("/home/amigos/NECST/script/data/obs_log/"+daystmp+".txt", "a")
    #f = open("./"+daystmp+".txt", "a")
    f = open("/tmp/"+daystmp+".txt", "a")
    f.write("\n")
    f.write("*** Initial Check")
    f.write("- Weather\n")
    f.write(" %s [UTC]\n" %(tstmp))
    f.write(" In Temp %s [C]\n" %(in_temp[2]))
    f.write(" Out Temp %s [C]\n" %(out_temp[0]))
    f.write(" Dome Temp %s [C]\n" %(d_temp[2]))
    f.write(" Cab Temp %s [C]\n" %(c_temp[2]))
    f.write(" In Humi %s [%s]\n" %(in_humi[0], "%"))
    f.write(" Out Humi %s [%s]\n" %(out_humi[0], "%"))
    f.write(" Wind Dir %s [deg]\n" %(wind_dir[0]))
    f.write(" Wind Sp %s [m/s]\n" %(wind_speed[2]))
    f.write(" Pressure %s [hPa]\n" %(press[0]))
    f.write("\n\n")
    f.close()

