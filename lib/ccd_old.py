from astropy.time import Time
import math
import time
#import pyinterface
import numpy as np
from PIL import Image
from PIL import ImageOps
import os
import rospy #debug all_sky_shot
import sys
sys.path.append('/home/amigos/ros/src/necst/scripts/device')
from datetime import datetime as dt



class ccd_controller(object):

    error = []
    status = ''
    error_count = 0
    
    
    def __init__(self):
        #open
        #self.img = pyinterface.create_gpg5520(1)
        return
    
    def print_msg(self,msg):
        print(msg)
        return
    
    def print_error(self,msg):
        self.error.append(msg)
        self.print_msg('!!!!ERROR!!!!' + msg)
        return
    
    def save_status(self, x, y, number, magnitude, az_star, el_star, dir_name, data_name, status):
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
        trim_image.save(dir_name+data_name+"_trim.jpg")
        ###triming end

        in_image = Image.open(dir_name+data_name+"_trim.jpg")
        image = np.array(ImageOps.grayscale(in_image))
        ori_image = np.array(image)
        #"""
        #threshold
        width = len(image[0])
        height = len(image)
        for i in range(height):
            for j in range(width):
                if image[i][j] < thr:
                    image[i][j] = 0
        
        #calc dimention
        p_array = np.zeros(256)
        for i in range(height):
            for j in range(width):
                p_array[image[i][j]] += 1
        
        #find color
        num = 1
        nmax = 1
        for i in range(255):
            if nmax < p_array[i+1]:
                nmax = p_array[i+1]
                num = i+1
        
        #find star
        x = 0
        y = 0
        n = 0
        for i in range(height):
            for j in range(width):
                if image[i][j] == num:
                    x += j
                    y += i
                    n += 1
        if n == 0:
            print("CAN'T FIND STAR") #black photograph
            return 1
        x = x/n
        y = y/n
        
        #find center
        xx = 0.
        yy = 0.
        f = 0.
        x = int(x)
        y = int(y)
        try:
            for i in range(21):
                for j in range(21):
                    xx += (x+j-10.)*image[y+i-10][x+j-10]
                    yy += (y+i-10.)*image[y+i-10][x+j-10]
                    f += image[y+i-10][x+j-10]
        except Exception as e:
            print(e)
            rospy.loginfo(e)
            xx = 10000
            yy = 10000
            f = 0
        
        if f == 0.: #two or more stars
            print("MANY STARS ARE PHOTOGRAPHED")
            return 2
        
        xx = xx/f
        yy = yy/f
        print(xx)
        print(yy)
        
        #self.save_status(xx, yy, number, magnitude, az_star, el_star, mjd, data_name, secofday, status) #move to ROS_all_sky_shot.py
        return [xx, yy]#"""
        pass
    
    #for tracking(test)
    def save_track_status(self, x, y, ra, dec, az_star, el_star, mjd, data_name, secofday, status):
        if os.path.exists("/home/nfs/necopt-old/ccd-shot/data/"+str(data_name)):
            pass
        else:
            os.mkdir("/home/nfs/necopt-old/ccd-shot/data/"+str(data_name))
            print("---------------------------------------------------------")
            print("---------------------------------------------------------")
            print("---------------------------------------------------------")
            print("---------------------------------------------------------")
        f = open("/home/nfs/necopt-old/ccd-shot/data/"+str(data_name)+"/param.log", "a")
        
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        
        
        #geo_status = [x1,x2,y1,y2] #for test
        geo_status = [0,0,0,0]#self.geo.read_geomech()
        geo_x = 0#(geo_status[0]-geo_status[1])/2
        geo_y = 0#(geo_status[2]-geo_status[3])/2
        geo_temp = [0,0]#self.geo.read_geomech_temp()
        
        #write papram
        f.write(str(ra)+" "+str(dec)+" "+str(mjd)+" "+str(secofday)+" "+str(status.Command_Az)+" "+str(status.Command_El)+" "\
        +str(status.Current_Az)+" "+str(status.Current_El)+" "+str(status.Current_Dome)+" "+str(x)+" "+str(y)+" "+str(status.OutTemp)+" "\
        +str(status.Press)+" "+str(status.OutHumi)+" "+str(az_star)+" "+str(el_star)+" "+str(status.WindDir)+" "+str(status.WindSp)+" "\
        +str(geo_x)+" "+str(geo_y)+" "+str(geo_status[0])+" "+str(geo_status[3])+" "+str(geo_temp[0])+" "+str(geo_status[1])+" "\
        +str(geo_status[3])+" "+str(geo_temp[1]))
        f.write("\n")
        f.close()
        return
    
    
    
    #for tracking(test)
    def onepoint_shot(self, ra, dec, az_star, el_star, data_name, status):
        thr = 80 #threshold of brightness <=?
        
        if os.path.exists("/home/nfs/necopt-old/ccd-shot/data/"+str(data_name)):
            pass
        else:
            print("create file")
            print("/home/nfs/necopt-old/ccd-shot/data/"+str(data_name))
            os.mkdir("/home/nfs/necopt-old/ccd-shot/data/"+str(data_name))
        
        name = time.strftime('%Y%m%d_%H%M%S')
        
        self.oneshot(data_name, name)
        date = dt.utcnow()
        mjd = Time(date).mjd
        secofday = date.hour*60*60 + date.minute*60 + date.second + date.microsecond*0.000001

        #?????
        """
        path = os.getcwd()
        com = "mv "+str(path)+"/"+str(name)+".bmp /home/amigos/NECST/soft/data/"+str(data_name)+"/"+str(name)+".bmp"
        ret = commands.getoutput(com)
        """
        
        #load array
        #print(ret)
        time.sleep(1.5)
        in_image = Image.open("/home/nfs/necopt-old/ccd-shot/data/"+str(data_name)+"/"+name+".bmp")
        image = np.array(ImageOps.grayscale(in_image))
        ori_image = np.array(image)
        
        #threshold
        width = len(image[0])
        height = len(image)
        for i in range(height):
            for j in range(width):
                if image[i][j] < thr:
                    image[i][j] = 0
        
        #calc dimention
        p_array = np.zeros(256)
        for i in range(height):
            for j in range(width):
                p_array[image[i][j]] += 1
        
        #find color
        num = 1
        nmax = 1
        for i in range(255):
            if nmax < p_array[i+1]:
                nmax = p_array[i+1]
                num = i+1
        
        #find star
        x = 0
        y = 0
        n = 0
        for i in range(height):
            for j in range(width):
                if image[i][j] == num:
                    x += j
                    y += i
                    n += 1
        if n == 0:
            print("CAN'T FIND STAR") #black photograph
            return 1
        x = x/n
        y = y/n
        
        #find center
        xx = 0.
        yy = 0.
        f = 0.
        x = int(x)
        y = int(y)
        for i in range(21):
            for j in range(21):
                xx += (x+j-10)*image[y+i-10][x+j-10]
                yy += (y+i-10)*image[y+i-10][x+j-10]
                f += image[y+i-10][x+j-10]
        
        if f == 0.: #two or more stars
            print("MANY STARS ARE PHOTOGRAPHED")
            return 1
        
        xx = xx/f
        yy = yy/f
        print("==============================================")
        print("==============================================")
        print("==============================================")
        print(xx)
        print(yy)

        self.save_track_status(xx, yy, ra, dec, az_star, el_star, mjd, data_name, secofday, status)
        return
        




