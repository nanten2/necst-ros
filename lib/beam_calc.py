#! /usr/bin/env python3

import numpy as np

class beam_calc(object):

    def __init__(self):
        pass

    def calc_model(self,*param):
        r, theta ,d = param
        result = r * np.sin(np.deg2rad((el / 3600) + theta)) + d
        return result
    
    def calc(self, center, az, el):

        if center == 1:
            param_az = [0,0,0]
            param_el = [0,0,0]

        elif center == 2:
            param_az = [0,0,0]
            param_el = [0,0,0]
            
        elif center == 3:
            param_az = [-296.25498697, 92.80852018, 0]
            param_el = [301.31394405, -2.24658021, 0]
            
        elif center == 4:
            param_az = [-300.30348947, -5.39832257, 0]
            param_el = [-356.60738273, 83.0761821, 0]
            
        elif center == 5:
            param_az = [387.92046571, 98.14590096, 0]
            param_el = [-341.72927682, 2.31592071, 0]

        ddx = self.calc_model(el, *param_az)
        ddy = self.calc_model(el, *param_el)
        return ddx,ddy

    def calc_list(self, center, x_list, y_list):
        _x_list = []
        _y_list = []
        for i in range(len(x_list)):
            ret = self.calc(center, x_list[i], y_list[i])
            x = x_list[i] - ret[0]
            y = y_list[i] - ret[1]
            _x_list.append(x)
            _y_list.append(y)

        return [_x_list, _y_list]
            
            
