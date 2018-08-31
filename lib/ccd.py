import datetime
from astropy.time import Time
import math
import time
import pyinterface
import numpy as np
from PIL import Image
from PIL import ImageOps
import os
#from pyslalib import slalib




class ccd_controller(object):

    error = []
    status = ''
    
    
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
    
    def oneshot(self, dirname, filename, frame_no = 1, size=640*480*3, Bufferformat='IFIMG_COLOR_RGB24', StartMode = 'IFIMG_DMACAPTURE_START', framenum = 1):
        """
        # set buffer
        self.img.set_format(frame_no, size, Bufferformat)
        #start cap
        self.img.start_cap(frame_no, StartMode)
        # get status
        status = self.img.get_status()
        # get data
        self.img.get_data(frame_no, framenum, size, dwDataFormat = 'IFIMG_COLOR_RGB24', dwXcoodinates = 0, dwYcoodinates = 0, dwXLength = 640, dwYLength = 480)
        # save data
        self.img.save(fiename, size*3, Bufferformat)
        return status
        """
        
        f = open("/home/nfs/necopt-old/ccd-shot/ccd-shot-command.txt", "w")
        f.write(str(dirname) + "/" + str(filename) + ".bmp")
        f.close()

        return
    
    def save_status(self, x, y, number, magnitude, az_star, el_star, mjd, data_name, secofday, status):

        f = open("/home/amigos/data/experiment/opt/"+str(data_name)+"/process.log", "a")
        
        #geo_status = [x1,x2,y1,y2] #for test
        geo_status = [0,0,0,0]
        geo_x = 0
        geo_y = 0
        geo_temp = 0
        
        #write papram
        f.write(str(number)+" "+str(magnitude)+" "+str(mjd)+" "+str(secofday)+" "+str(status.Command_Az)+" "+str(status.Command_El)+" "\
        +str(status.Current_Az)+" "+str(status.Current_El)+" "+str(status.Current_Dome)+" "+str(x)+" "+str(y)+" "+str(status.OutTemp)+" "+str(status.Press)\
        +" "+str(status.OutHumi)+" "+str(az_star)+" "+str(el_star)+" "+str(geo_x)+" "+str(geo_y)+" "+str(geo_status[0])+" "+str(geo_status[1])\
        +" "+str(geo_temp[0])+" "+str(geo_status[2])+" "+str(geo_status[3])+" "+str(geo_temp[1]))
        f.write("\n")
        f.close()
        return
    
    def all_sky_shot(self, number, magnitude, az_star, el_star, data_name, status):
        thr = 80 #threshold of brightness
        
        #status = {"Command_Az":0,"Command_El":0,"Current_Az":0,"Current_El":0,"OutTemp":0,"Press":0,"OutHumi":0}
        
        date = datetime.datetime.today()
        month = str("{0:02d}".format(date.month))
        day = str("{0:02d}".format(date.day))
        hour = str("{0:02d}".format(date.hour))
        minute = str("{0:02d}".format(date.minute))
        second = str("{0:02d}".format(date.second))
        name = str(date.year)+month+day+hour+minute+second
        
        #oneshot
        self.oneshot(data_name,name)
        mjd = Time(date).mjd
        secofday = date.hour*60*60 + date.minute*60 + date.second + date.microsecond*0.000001
        
        #load array
        #path = os.getcwd()
        #com = "mv "+str(path)+"/"+str(name)+".bmp /home/amigos/NECST/soft/data/"+str(data_name)+"/"+str(name)+".bmp"
        #ret = commands.getoutput(com)
        #print(ret)

        time.sleep(3.)# wait saving bmp

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
        for i in range(21):
            for j in range(21):
                xx += (x+j-10.)*image[y+i-10.][x+j-10.]
                yy += (y+i-10.)*image[y+i-10.][x+j-10.]
                f += image[y+i-10.][x+j-10.]
        
        if f == 0.: #two or more stars
            print("MANY STARS ARE PHOTOGRAPHED")
            return 2
        
        xx = xx/f
        yy = yy/f
        print(xx)
        print(yy)
        
        self.save_status(xx, yy, number, magnitude, az_star, el_star, mjd, data_name, secofday, status)
        return [xx, yy]
    
    
    
    
    
    
    #for tracking(test)
    def save_track_status(self, x, y, ra, dec, az_star, el_star, mjd, data_name, secofday, status):
        if os.path.exists("/home/amigos/NECST/soft/core/"+str(data_name)):
            pass
        else:
            os.mkdir("/home/amigos/NECST/soft/core/"+str(data_name))
            print("---------------------------------------------------------")
            print("---------------------------------------------------------")
            print("---------------------------------------------------------")
            print("---------------------------------------------------------")
        f = open("/home/amigos/NECST/soft/core/"+str(data_name)+"/param.log", "a")
        
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        
        
        #geo_status = [x1,x2,y1,y2] #for test
        geo_status = self.geo.read_geomech()
        geo_x = (geo_status[0]-geo_status[1])/2
        geo_y = (geo_status[2]-geo_status[3])/2
        geo_temp = self.geo.read_geomech_temp()
        
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
        
        if os.path.exists("/home/amigos/NECST/soft/data/"+str(data_name)):
            pass
        else:
            os.mkdir("/home/amigos/NECST/soft/data/"+str(data_name))
        
        name = time.strftime('%Y%m%d_%H%M%S')
        
        #oneshot
        self.oneshot(name)
        date = datetime.datetime.today()
        ret = slalib.sla_cldj(date.year, date.month, date.day)
        mjd = ret[0]
        secofday = date.hour*60*60 + date.minute*60 + date.second + date.microsecond*0.000001
        
        path = os.getcwd()
        com = "mv "+str(path)+"/"+str(name)+".bmp /home/amigos/NECST/soft/data/"+str(data_name)+"/"+str(name)+".bmp"
        ret = commands.getoutput(com)
        
        #load array
        print(ret)
        in_image = Image.open("/home/amigos/NECST/soft/data/"+str(data_name)+"/"+name+".bmp")
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
        for i in range(21):
            for j in range(21):
                xx += (x+j-10.)*image[y+i-10.][x+j-10.]
                yy += (y+i-10.)*image[y+i-10.][x+j-10.]
                f += image[y+i-10.][x+j-10.]
        
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
        




