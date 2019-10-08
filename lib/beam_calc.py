#! /usr/bin/env python3

import numpy as np

class beam_calc(object):

    def __init__(self):
        pass

    def calc(self, center, az, el):
        if center == 1:
            ddx = 0
            ddy = 0

        elif center == 2:
            ddx = 60*5.35*(-np.sin(np.radians(-el)))
            ddy = 60*5.35*(np.cos(np.radians(-el)))
            
        elif center == 3:
            ddx = - 60*5.35*(np.cos(np.radians(-el)))
            ddy = - 60*5.35*(np.sin(np.radians(-el)))
            
        elif center == 4:
            ddx = - 60*5.35*(-np.sin(np.radians(-el)))
            ddy = - 60*5.35*(np.cos(np.radians(-el)))
            
        elif center == 5:
            ddx = 60*5.35*(np.cos(np.radians(-el)))
            ddy = 60*5.35*(np.sin(np.radians(-el)))

        return ddx,ddy

    def calc_list(self, center, x_list, y_list):
        _x_list = []
        _y_list = []
        for i in range(len(x_list)):
            ret = self.calc(center, x_list[i], y_list[i])
            x = x_list[i] - ddx
            y = y_list[i] - ddy
            _x_list.append(x)
            _y_list.append(y)

        return _x_list,_y_list
            
            
