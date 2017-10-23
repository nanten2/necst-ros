import time
import math
import azel_calc
from datetime import datetime as date



class otf(object):

    def __init__(self):
        
        self.azel = azel_calc.azel_calc()

    def otf_scan(self, lambda_on, beta_on, dcos, coord_sys, dx, dy, dt, num, rampt, delay, lamda, hosei, code_mode, off_x, off_y, offcoord, temp, press, humi):
        print(lambda_on, beta_on, dcos, coord_sys, dx, dy, dt, num, rampt, \
delay, lamda, hosei, code_mode, off_x, off_y, offcoord, temp, press, humi)
        start_x = off_x-float(dx)/2.-float(dx)/float(dt)*rampt
        start_y = off_y-float(dy)/2.-float(dy)/float(dt)*rampt
        total_t = rampt + dt * num
        end_x = off_x + dx * (num - 0.5)
        end_y = off_y + dy * (num - 0.5)
        obs_start = time.time() + delay
        obs_end = obs_start + total_t
        off_dx_vel = (end_x - start_x) / (obs_end - obs_start)
        off_dy_vel = (end_y - start_y) / (obs_end - obs_start)
        print("off_dx_vel : ", off_dx_vel)
        print("off_dy_vel : ", off_dy_vel)
        nt = 0
        az_list = []
        el_list = []
        now = date.now()
        while nt < total_t:
            print(dt)
            print("off_x", start_x+off_dx_vel*nt)
            print("off_y", start_y+off_dy_vel*nt)
            ret = self.azel.coordinate_calc(lambda_on, beta_on, num, code_mode, 
                            start_x+off_dx_vel*nt, start_y+off_dy_vel*nt, offcoord, 
                            hosei, lamda, dcos, temp, press,
                            humi, now, loop=1, time_rate=nt,
                            )
            az_list.append(ret[0][0])
            el_list.append(ret[1][0])
            nt += 0.1
            print(ret[0][0],ret[1][0])

        start_time = float(now.strftime("%s"))+9*3600. + float(now.strftime("%f"))*1e-6
        print("az : ", az_list)
        print("el : ", el_list)
        print("time : ", start_time)
        return [az_list, el_list, start_time]
        
