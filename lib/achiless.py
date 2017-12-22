#! /usr/bin/env python3

import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
import ac240

class dfs(object):
    
    def __init__(self):
        host_dfs01 = '172.20.0.41'
        port_dfs01 = 52700
        host_dfs02 = '172.20.0.43'
        port_dfs02 = 52701
        self.dfs01 = ac240.ac240(host_dfs01, port_dfs01)
        self.dfs02 = ac240.ac240(host_dfs02, port_dfs02)
        self.dfs01.open()
        self.dfs02.open()
        pass
    
    def oneshot_dfs01(self, repeat=1, integsec=1.0, starttime=0.0):
        if type(repeat) == tuple:
            starttime = repeat[2]
            integsec = repeat[1]
            repeat = repeat[0]
            pass
        
        self.dfs01.getspectrum(repeat, integsec, starttime)
        data = self.dfs01.getdata()
        return data
    
    def oneshot_dfs02(self, repeat=1, integsec=1.0, starttime=0.0):
        if type(repeat) == tuple:
            starttime = repeat[2]
            integsec = repeat[1]
            repeat = repeat[0]
            pass
        
        self.dfs02.getspectrum(repeat, integsec, starttime)
        data = self.dfs02.getdata()
        return data
    
    def oneshot(self, repeat=1, integsec=1.0, starttime=0.0):
        arg = (repeat, integsec, starttime)
        args = (arg, arg)
        funcs = (self.oneshot_dfs01, self.oneshot_dfs02)
        data = tmap(funcs, args)
        return data
