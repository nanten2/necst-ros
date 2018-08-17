#! /usr/bin/env python3

import rospy
from necst.msg import List_coord_msg
from necst.msg import Status_weather_msg
from necst.msg import Bool_necst
from necst.msg import String_necst
from necst.msg import Float64_necst

from datetime import datetime
from astropy.time import Time
import time
from scipy.optimize import curve_fit
import sys
sys.path.append("/home/amigos/ros/src/necst/lib/")
sys.path.append("/home/necst/ros/src/necst/lib/")
import calc_coord


node_name = "azel_list"


class azel_list(object):

    weather = ""
    param = ""
    stop_flag = False
    old_list = ""
    accele = 1
    
    def __init__(self):
        self.start_time = time.time()
        rospy.Subscriber("wc_list", List_coord_msg, self._receive_list, queue_size=1)
        rospy.Subscriber("status_weather", Status_weather_msg, self._receive_weather, queue_size=1)
        rospy.Subscriber("move_stop", Bool_necst, self._stop, queue_size=1)
        
        self.pub = rospy.Publisher("list_azel", List_coord_msg, queue_size=1)
        self.stop = rospy.Publisher("move_stop", Bool_necst, queue_size=1)
        self.obs_stop = rospy.Publisher("obs_stop", String_necst, queue_size=1)
        self.pub_accele = rospy.Publisher("accele", Float64_necst, queue_size=1)
        rospy.Subscriber("accele_speed", Float64_necst, self._receive_accele, queue_size=1)        
        self.msg = Bool_necst()
        self.msg.from_node = node_name
        self.calc = calc_coord.azel_calc()
        pass

    def _receive_accele(self, req):
        print("accele speed : ", req.data)
        self.accele = req.data
        return
    
    
    def _receive_weather(self, req):
        self.weather = req
        return
    
    def _receive_list(self, req):
        ### x,y is [arcsec]
        print("list")
        print(req)
        if req.timestamp < self.start_time:
            print("receive_old_list...")
        else:
            self.stop_flag = False
            self.param = req
            pass
        return

    '''
    def linear_fit(self,x, a, b):
        return a*x+b

    def curve2_fit(self, x, a, b):
        return a*x**2+b*x+self.p0
    '''    

    def create_azel_list(self):
        msg = List_coord_msg()
        print("wait comming list...")
        while (self.param =="") and (not rospy.is_shutdown()) :
            time.sleep(0.1)
        print("start_calclation!!")
        loop = 0
        check = 0
        param = self.param        
        while not rospy.is_shutdown():
            if not self.param:
                time.sleep(1.)
                continue
            elif param != self.param:
                loop = 0
                check = 0
                param = self.param
            else:
                pass
            
            if self.stop_flag == False:
                if len(param.x_list) > 2:
                    dt = 0.1                    

                    # linear fitting
                    len_x = param.x_list[loop+1] - param.x_list[loop]
                    len_y = param.y_list[loop+1] - param.y_list[loop]
                    len_t = param.time_list[loop+1] - param.time_list[loop]
                
                    dx = len_x/(len_t*10)#[arcsec/100ms]
                    dy = len_y/(len_t*10)#[arcsec/100ms]
                    dt = 0.1

                    x_list2 = [param.x_list[loop] + dx*(i+check*100) for i in range(100)]
                    y_list2 = [param.y_list[loop] + dy*(i+check*100) for i in range(100)]
                    time_list2 = [param.time_list[loop]+dt*(i+check*100) for i in range(100)]
                    loop_count = 0
                    check_count = 1
                    for i in range(100):
                        if param.time_list[-1]< time_list2[-1]:
                            del x_list2[-1]
                            del y_list2[-1]
                            del time_list2[-1]
                            self.stop_flag = True
                        elif param.time_list[loop+1] <= time_list2[-1]:
                            del x_list2[-1]
                            del y_list2[-1]
                            del time_list2[-1]
                            loop_count = 1
                            check = 0
                            check_count = 0
                        else:
                            break
                    loop += loop_count                        
                    if loop == len(param.time_list)-1:
                        self.stop_flag = True                        
                    check +=  check_count
                    """ debug
                    plt.plot(param.time_list, param.x_list,label="target",linestyle='None',marker='.')
                    plt.plot(time_list2, x_list2,label="v2",linestyle='None',marker='.')
                    plt.show()
                    """

                else:
                    len_x = param.x_list[1] - param.x_list[0]
                    len_y = param.y_list[1] - param.y_list[0]
                    len_t = param.time_list[1] - param.time_list[0]

                    dx = len_x/(len_t*10)#[arcsec/100ms]
                    dy = len_y/(len_t*10)#[arcsec/100ms]
                    dt = 0.1
                
                    x_list2 = [param.x_list[0] + dx*(i+loop*100) for i in range(100)]
                    y_list2 = [param.y_list[0] + dy*(i+loop*100) for i in range(100)]
                    time_list2 = [param.time_list[0]+dt*(i+loop*100) for i in range(100)]
                    loop += 1
                    
                    for i in range(100):
                        if param.time_list[-1]< time_list2[-1]:
                            del x_list2[-1]
                            del y_list2[-1]
                            del time_list2[-1]
                            self.stop_flag = True
                        else:
                            break

                if time_list2 != []:
                    time_list3 = [datetime.fromtimestamp(time_list2[i]) for i in range(len(time_list2))]
                    astro_time = Time(time_list3)

                    
                    ret = self.calc.coordinate_calc(x_list2, y_list2, astro_time,
                                                    param.coord, param.off_az, param.off_el, 
                                                    param.hosei, param.lamda, self.weather.press,
                                                    self.weather.out_temp, self.weather.out_humi, param.limit)

                    
                    ret[0] = self.negative_change(ret[0])

                else:
                    limit_flag = True
                    
                """limit check"""
                for i in range(len(time_list2)):
                    if not -240*3600<ret[0][i]<240*3600 or not 0.<=ret[1][i]<80*3600.:
                        print("reaching soft limit : ", )
                        print("az : ", ret[0][i]/3600., "[deg]")
                        print("el : ", ret[1][i]/3600., "[deg]")
                        self.stop_flag = True
                        limit_flag = True
                        data = "reaching soft limit : (az, el)=("+str(ret[0][i]/3600.)+", "+str(ret[1][i]/3600.)+") [deg]"
                        self.obs_stop.publish(data = data, from_node=node_name, timestamp=time.time())                        
                        break
                    else:
                        pass
                    limit_flag = False
                if not limit_flag:
                    self.msg.timestamp = time.time()
                    self.msg.data = False
                    self.stop.publish(self.msg)
                    msg.x_list = ret[0]
                    msg.y_list = ret[1]
                    msg.time_list = time_list2
                    msg.from_node =node_name
                    msg.timestamp = time.time()
                    self.pub.publish(msg)
                    #print("msg", msg)
                    if param.from_node != "worldcoordinate_otf":
                        print("#### accele!! ####")
                        self.pub_accele.publish(self.accele,node_name,time.time())
                    else:
                        pass
                    #print("publish ok")
                else:
                    limit_flag = False
            else:
                print("###########################")
                if param.from_node == "worldcoordinate_otf":
                    print("#### accele!! ####")
                    self.pub_accele.publish(self.accele,node_name,time.time())
                else:
                    pass
                loop = 0
                self.param = ""
                pass
            
            time.sleep(0.1)
        return

    def _stop(self,req):
        if req.data == True:
            self.stop_flag = req.data
        else:
            pass
        return

    def negative_change(self, az_list):
        if all((-240*3600<i< 240*3600. for i in az_list)):
            pass
        elif all((i<-110*3600. for i in az_list)):
            az_list = [i+360*3600. for i in az_list]
        elif all((i>110*3600. for i in az_list)):
            az_list = [i-360*3600. for i in az_list]
        elif all((-270*3600<i< 270*3600. for i in az_list)):
            pass
        else:
            print("Az limit error.")
        return az_list
        
if __name__ == "__main__":
    rospy.init_node(node_name)
    azel = azel_list()
    azel.create_azel_list()

