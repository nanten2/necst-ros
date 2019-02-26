import time
import numpy as np
from PIL import Image
from PIL import ImageOps
import os
import rospy
import sys
sys.path.append('/home/amigos/ros/src/necst/scripts/device')
from datetime import datetime as dt

class ccd_controller(object):

    def __init__(self):
        return
    
    def save_status(self, x, y, number, magnitude, az_star, el_star, dir_name, data_name, status):
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)
        f = open(dir_name+"process.log", "a")#shiotani changed save path

        geo_x=geo_y=0
        geo_status=geo_temp=[0,0,0,0]
        
        #write papram
        f.write(str(number)+" "+str(magnitude)+" "+str(status.MJD)+" "+str(status.Secofday)+" "+str(status.Command_Az)+" "+str(status.Command_El)+" "\
        +str(status.Current_Az)+" "+str(status.Current_El)+" "+str(status.Current_Dome)+" "+str(x)+" "+str(y)+" "+str(status.OutTemp)+" "+str(status.Press)\
        +" "+str(status.OutHumi)+" "+str(az_star)+" "+str(el_star)+" "+str(geo_x)+" "+str(geo_y)+" "+str(geo_status[0])+" "+str(geo_status[1])\
        +" "+str(geo_temp[0])+" "+str(geo_status[2])+" "+str(geo_status[3])+" "+str(geo_temp[1])+"\n")
        f.close()
        return

    def save_track_status(self, x, y, ra, dec, az_star, el_star, dir_name, data_name, status):
        f = open(dir_name+"process.log", "a")#shiotani changed save path

        geo_x=geo_y=0
        geo_status=geo_temp=[0,0,0,0]
            
        #write papram
        f.write(str(number)+" "+str(magnitude)+" "+str(status.MJD)+" "+str(status.Secofday)+" "+str(status.Command_Az)+" "+str(status.Command_El)+" "\
        +str(status.Current_Az)+" "+str(status.Current_El)+" "+str(status.Current_Dome)+" "+str(x)+" "+str(y)+" "+str(status.OutTemp)+" "+str(status.Press)\
        +" "+str(status.OutHumi)+" "+str(az_star)+" "+str(el_star)+" "+str(geo_x)+" "+str(geo_y)+" "+str(geo_status[0])+" "+str(geo_status[1])\
        +" "+str(geo_temp[0])+" "+str(geo_status[2])+" "+str(geo_status[3])+" "+str(geo_temp[1])+"\n")
        f.close()
        return
    
    
    def ccd_analysis(self, data_name, dir_name,):
        thr = 80 #threshold of brightness
        
        print("wait for saving data...")
        while not rospy.is_shutdown():
            if os.path.exists(dir_name+data_name+".jpg") == True:
                break
            time.sleep(0.1)

        ###triming
        origin_image = Image.open(dir_name+data_name+".jpg")#shiotani added
        trim_image = origin_image.crop((2080.0, 1360.0, 2720.0, 1840.0))
        trim_dir = dir_name.rsplit("/",2)[0]+"/trim/"
        if not os.path.exists(trim_dir):
            os.makedirs(trim_dir)
        trim_image.save(trim_dir+data_name+"_trim.jpg")
        ###triming end

        in_image = Image.open(trim_dir+data_name+"_trim.jpg")
        image = np.array(ImageOps.grayscale(in_image))
        ori_image = np.array(image)

        ###
        image[image < thr] = 0
        
        #mode
        bins = np.bincount(image.flatten())
        num = np.argmax(bins[1:]) +1
        nmax = np.max(bins[1:])
        x_mom1 = 0
        y_mom1 = 0
        
        if nmax == 0:
            print("CAN'T FIND STAR") #black photograph
            return ["ERROR","Can't find star!!"]
            
        else:
            #find center
            y, x = (np.sum(np.where(image == num), axis=1) / nmax).astype(np.int)
            f = np.sum(image[y-10:y+10,x-10:x+10])
            if f == 0.: #two or more stars
                print("MANY STARS ARE PHOTOGRAPHED")
                return ["ERROR", "Many stars are photographed!!"]
            else:
                for i, _l in enumerate(image[y-10:y+10,x-10:x+10]):
                    for j, _a in enumerate(_l):
                        x_mom1 += (x+j-10) * _a
                        y_mom1 += (y+i-10) * _a
                xx = x_mom1 / f
                yy = y_mom1 / f
                print(xx)
                print(yy)
        
                return [xx, yy]
    
