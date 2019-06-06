# coding: utf-8

# ## hosei_opt.txt を向上させると思われるヤツ

import os
import sys
import glob
import numpy
import pandas
import matplotlib
#get_ipython().magic('matplotlib inline')
import matplotlib.pyplot as plt
plt.rcParams['figure.facecolor'] = 'white'
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d import Axes3D
import scipy.optimize

import plot
import plot_3d


cos = numpy.cos
sin = numpy.sin
d2r = numpy.deg2rad
r2d = numpy.rad2deg
diag = numpy.diag
sqrt = numpy.sqrt

hosei_path = '/home/amigos/pointing/data/opt_data/opt_20190219/opt_20190219085938/hosei_opt.txt'

def hosei_point(param_dir):
    ind_az = 14
    ind_el = 15
    ind_dx = 9
    ind_dy = 10
    az_list, el_list, dx_list, dy_list = [], [], [], []
    for _ in param_dir:
        d = numpy.loadtxt(_ + '/process.log')
        d[:,ind_dx]=(d[:,ind_dx]-320)*1.762
        d[:,ind_dy]=(d[:,ind_dy]-240)*1.762
        az_list.append(d[:,ind_az])
        el_list.append(d[:,ind_el])
        dx_list.append(d[:,ind_dx])
        dy_list.append(d[:,ind_dy])    

### process.log を plot
    plot.plot(az_list, el_list, dx_list, dy_list, param_dir)


# ### azel 平面内に dx, dy を plot する

    plot_3d.plot(az_list, el_list, dx_list, dy_list)


# ## dx と dy の model との差の二乗和を最小にするような器差を求める

# ### 器差モデルの定義


    def func(kisa, az, el, dx, dy):
    
        def model_dx(az, el, kisa):
            dx = kisa[2] * sin(d2r(kisa[3] - az)) * sin(d2r(el))         + kisa[4] * sin(d2r(el))         + kisa[0] * cos(d2r(el))         + kisa[1] 
            return dx

        def model_dy(az, el, kisa):
            dy = - kisa[2] * cos(d2r(kisa[3] - az))         + kisa[15]         + kisa[11] * numpy.array(el) * 3600. 
            return dy
        
        residual_dx = model_dx(az, el, kisa) - dx
        residual_dy = model_dy(az, el, kisa) - dy
    
        return sqrt(residual_dx ** 2 + residual_dy ** 2)


# ### フィッティング初期値の定義

# initial params.
    kisa0 = [0.] * 24

# Azエンコーダーオフセット (arcsec.)
    kisa0[0] = 0.

# Az オフセット (arcsec.)
    kisa0[1] =  0.

# Az 軸の倒れ 振幅 (arcsec.)
    kisa0[2] =  40.

# Az 軸の倒れ 位相 (deg.)
    kisa0[3] = 90.

# Az 軸と El 軸の非直行性 (arcsec.)
    kisa0[4] = 0.

# 重力・大気差 1次・光学用 (const.)
    kisa0[11] = 0.

# El 方向 の offset (arcsec.)
    kisa0[15] = 0.


# ### フィッティング実行実行

    az = [e for inner_list in az_list for e in inner_list]
    el = [e for inner_list in el_list for e in inner_list]
    dx = [e for inner_list in dx_list for e in inner_list]
    dy = [e for inner_list in dy_list for e in inner_list]
    kisa = scipy.optimize.leastsq(func, kisa0, args=(az, el, dx, dy) ,maxfev=1000000)[0]
# kisa = scipy.optimize.leastsq(func, kisa0, args=(az, el, dx, dy) )[0]


# ### plot 用のモデルの定義


    def model_dx(az, el, *kisa):
        dx = kisa[2] * sin(d2r(kisa[3] - az)) * sin(d2r(el))     + kisa[4] * sin(d2r(el))     + kisa[0] * cos(d2r(el))     + kisa[1] 
        return dx

    def model_dy(az, el, *kisa):
        dy = - kisa[2] * cos(d2r(kisa[3] - az))     + kisa[15]     + kisa[11] * numpy.array(el) * 3600.     
        return dy


# ### 測定データと fitting モデルを azel 空間に plot
 
    nrow = 1
    ncol = 2
    nax = ncol * nrow
    figsize = (ncol * 8, nrow * 6)
    matplotlib.rcParams['font.size'] = 12
    
    fig = plt.figure(figsize=figsize)
    ax = [fig.add_subplot(nrow, ncol, i+1, projection='3d') for i in range(nax)]
    
    _az = numpy.linspace(0., 360., 11)
    _el = numpy.linspace(30., 80., 11)
    AZ, EL = numpy.meshgrid(_az, _el)
    _dx = model_dx(AZ, EL, *kisa)
    _dy = model_dy(AZ, EL, *kisa)

    for az, el, dx, dy in zip(az_list, el_list, dx_list, dy_list):
        ax[0].scatter(az, el, dx, label='raw', c='steelblue')
        ax[1].scatter(az, el, dy, label='raw', c='steelblue')

        ax[0].plot_wireframe(AZ, EL, _dx, label='model', color='blue', linewidth=0.3)
        ax[0].set_xlabel('Az [deg.]')
        ax[0].set_ylabel('El [deg.]')
        ax[0].set_zlabel('dx [arcsec.]')
        ax[0].set_title('Az vs El vs dx')

        ax[1].plot_wireframe(AZ, EL, _dy, label='model', color='blue', linewidth=0.3)
        ax[1].set_xlabel('Az [deg.]')
        ax[1].set_ylabel('El [deg.]')
        ax[1].set_zlabel('dy [arcsec.]')
        ax[1].set_title('Az vs El vs dy')

        plt.show()


# ### 残渣を 3D 上に plot

        new_dx_list = [numpy.array(dx) - model_dx(az, el, *kisa) 
                       for az, el, dx, dy in zip(az_list, el_list, dx_list, dy_list)]
        new_dy_list = [numpy.array(dy) - model_dy(az, el, *kisa) 
                       for az, el, dx, dy in zip(az_list, el_list, dx_list, dy_list)]
        
        nrow = 1
        ncol = 2
        nax = ncol * nrow
        figsize = (ncol * 8, nrow * 6)
        matplotlib.rcParams['font.size'] = 12

        fig = plt.figure(figsize=figsize)
        ax = [fig.add_subplot(nrow, ncol, i+1, projection='3d') for i in range(nax)]

        _az = numpy.linspace(0., 360., 11)
        _el = numpy.linspace(30., 80., 11)
        AZ, EL = numpy.meshgrid(_az, _el)
        _dx = model_dx(AZ, EL, *kisa) * 0.
        _dy = model_dy(AZ, EL, *kisa) * 0.

        for az, el, dx, dy in zip(az_list, el_list, new_dx_list, new_dy_list):
            ax[0].scatter(az, el, dx, label='raw', c='steelblue')
            ax[1].scatter(az, el, dy, label='raw', c='steelblue')

            ax[0].plot_wireframe(AZ, EL, _dx, label='model', color='blue', linewidth=0.3)
            ax[0].set_xlabel('Az [deg.]')
            ax[0].set_ylabel('El [deg.]')
            ax[0].set_zlabel('dx [arcsec.]')
            ax[0].set_title('Az vs El vs dx')

            ax[1].plot_wireframe(AZ, EL, _dy, label='model', color='blue', linewidth=0.3)
            ax[1].set_xlabel('Az [deg.]')
            ax[1].set_ylabel('El [deg.]')
            ax[1].set_zlabel('dy [arcsec.]')
            ax[1].set_title('Az vs El vs dy')


# ### az と el 方向に射影

# ### モデルと測定データの差分

            plot.plot(az_list, el_list, new_dx_list, new_dy_list, param_dir, raw=False)


            old_kisa = [float(_kisa) for _kisa in open(hosei_path)]
            kisa
            new_kisa = [0.] * 24


# ### 使用している器差パラメーター
# kisa[0]：エンコーダーオフセット（arcsec.）<br>
# kisa[1]：Az オフセット（arcsec.）<br>
# kisa[2]：Az 軸の倒れ 振幅（arcsec.）<br>
# kisa[3]：Az 軸の倒れ 位相（deg.）<br>
# kisa[4]：Az 軸と El 軸の非直交性（arcsec.）<br>
# kisa[11]：重力・大気差 1次・光学用 (const.)<br>
# kisa[15]：El 方向 の offset (arcsec.)<br>

# ### 足せばいいやつ
# エンコーダーオフセット
            new_kisa[0] = old_kisa[0] + kisa[0]

# Az オフセット
            new_kisa[1] = old_kisa[1] + kisa[1]

# Az 軸と El 軸の非直交性
            new_kisa[3] = old_kisa[3] + kisa[3]

# 重力・大気差 1次・光学用
            new_kisa[11] = old_kisa[11] + kisa[11]

# El 方向 の offset
            new_kisa[15] = old_kisa[15] + kisa[15]


# ### 三角関数の足し算
# Az 軸の倒れ
            A = abs(old_kisa[2] * cos(d2r(old_kisa[3])) + kisa[2] * cos(d2r(kisa[3])))
            B = abs(old_kisa[2] * sin(d2r(old_kisa[3])) + kisa[2] * sin(d2r(kisa[3])))
            amp = sqrt(A ** 2 + B ** 2)
            phi = numpy.arctan(B / A)

            new_kisa[2] = amp
            new_kisa[3] = r2d(phi)


# ## 器差パラメーター まとめ

            index = [
                'Enc. offset',
                'Az offset',
                'Az Shaft collapse amplitude',
                'Az Axis collapse phase',
                'Az / El Non-orthogonality',
                'Gravity atmosphere Primary term',
                'El offset'
                ]
            df = pandas.DataFrame(index=index)
            df['dim.'] = [
                'arcsec.',
                'arcsec.',
                'arcsec.',
                'deg.',
                'arcsec.',
                'const.',
                'arcsec.',
                ]
            ind = [0, 1, 2, 3, 4, 11, 15]
            old = [old_kisa[_ind] for _ind in ind]
            new = [new_kisa[_ind] for _ind in ind]
            diff = [_old - _new for _old, _new in zip(old, new)]
            df['old'] = old
            df['new'] = new
            df['diff'] = diff
            
            df
        



