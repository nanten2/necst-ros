import rospy
from necst.msg import List_coord_msg
from necst.msg import Status_weather_msg
from std_msgs.msg import Bool
from datetime import datetime
from astropy.time import Time
import time
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

    def __init__(self):
        self.start_time = time.time()
        self.sub = rospy.Subscriber("wc_list", List_coord_msg, self.receive_list, queue_size=1)
        self.sub = rospy.Subscriber("status_weather", Status_weather_msg, self.receive_weather, queue_size=1)
        self.pub = rospy.Publisher("list_azel", List_coord_msg, queue_size=1)
        #self.stop = rospy.Publisher("move_stop", Bool, queue_size=1)
        self.stop = rospy.Subscriber("move_stop", Bool, self.stop, queue_size=1)
        self.calc = calc_coord.azel_calc()
        pass

    def receive_weather(self, req):
        self.weather = req
    
    def receive_list(self, req):
        if req.timestamp < self.start_time:
            print("receive_old_list...")
        else:
            self.param = req
            pass
        #if req == self.old_list:
            #pass
        #else:
            #self.stop_flag = True
            #self.stop.publish(True)
            #time.sleep(0.5)
            #self.stop_flag = False
            #self.stop.publish(False)
            #self.param = req
            #self.old_list = req
        return

    
    def create_azel_list(self):
        msg = List_coord_msg()
        print("wait comming list...")
        while (self.param =="") and (not rospy.is_shutdown()) :
            time.sleep(0.1)
        print("start_calclation!!")
        loop = 0
        param = self.param        
        while not rospy.is_shutdown():
            if not self.param:
                time.sleep(1.)
                continue
            if param != self.param:
                loop = 0
                param = self.param
            else:
                pass
            if self.stop_flag == False:
                len_x = param.x_list[1] - param.x_list[0]
                len_y = param.y_list[1] - param.y_list[0]
                len_t = param.time_list[1] - param.time_list[0]
                
                dx = len_x/(len_t*10)#[arcsec/100ms]
                dy = len_y/(len_t*10)#[arcsec/100ms]
                dt = 0.1
                
                x_list2 = [param.x_list[0]+dx*(i+loop*10) for i in range(10)]
                y_list2 = [param.y_list[0]+dy*(i+loop*10) for i in range(10)]
                time_list2 = [param.time_list[0]+dt*(i+loop*10) for i in range(10)]
                for i in range(10):
                    if param.time_list[1]< time_list2[-1]:
                        del x_list2[-1]
                        del y_list2[-1]
                        del time_list2[-1]
                        self.stop_flag = True
                    else:
                        break
                time_list3 = [datetime.fromtimestamp(time_list2[i]) for i in range(len(time_list2))]
                astro_time = Time(time_list3)
                ret = self.calc.coordinate_calc(x_list2, y_list2, astro_time,
                                                param.coord, param.off_az, param.off_el, 
                                                param.hosei, param.lamda, self.weather.press,
                                                self.weather.out_temp, self.weather.out_humi, param.limit)
                msg.x_list = ret[0]
                msg.y_list = ret[1]
                msg.time_list = time_list2
                msg.from_node =node_name
                msg.timestamp = time.time()
                self.pub.publish(msg)
                print(msg)
                loop += 1
            else:                
                loop = 0
                self.param = ""
                pass
            time.sleep(0.1)
        return

    def stop(self,req):
        self.stop_flag = req.data
    
if __name__ == "__main__":
    rospy.init_node(node_name)
    azel = azel_list()
    azel.create_azel_list()

