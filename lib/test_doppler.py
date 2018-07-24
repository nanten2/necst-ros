#! /usr/bin/env python3
#-*- coding: utf-8 -*-

from astropy.time import Time
import astropy.io.fits as fits
import math
import sys
import numpy as np

sys.path.append("/home/amigos/necst/lib/")
import doppler_nanten
import doppler_nanten_ast

dir1 = '/home/amigos/ros/src/data/'


dp = doppler_nanten.doppler_nanten()
dpa = doppler_nanten_ast.doppler_nanten()

data = []
for j in range(-90, 100 ,10):
    tmp = []
    print("calculating DEC_{} ....".format(j))

    for i in range(0, 370, 10):
        jd = Time.now().jd
        
        vobs_dp = dp.calc_vobs(jd, math.radians(i), math.radians(j))
        dpa.t = Time.now()
        vobs_dpa = dpa.calc_vobs(math.radians(i), math.radians(j))
        vobs = vobs_dp - vobs_dpa
        tmp = np.append(tmp,vobs)
    
    data = np.append(data, tmp, axis = 0)

data = np.reshape(data, (19, 37))

hdu = fits.PrimaryHDU(data)
hdulist = fits.HDUList([hdu])
hdulist.writeto(dir1+'doppler_test.fits', clobber=True)
"""
print("Max", np.max(data))
print("Min", np.min(data))
print("Mean", np.mean(data))
print("Std", np.std(data))
print("var", np.var(data))
"""
