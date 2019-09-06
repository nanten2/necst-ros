import os
import sys
import numpy
import pandas
import matplotlib.pyplot as plt
plt.rcParams['figure.facecolor'] = 'white'
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d import Axes3D
import scipy.optimize

cos = numpy.cos
sin = numpy.sin
d2r = numpy.deg2rad
r2d = numpy.rad2deg
diag = numpy.diag
sqrt = numpy.sqrt



result = numpy.loadtxt('',usecols=(1,2,4,5,6,8,9,10))

hosei_path = '/home/amigos/data/opt_data/hosei_230.txt'

az_list_1, el_list_1, az_list_2, el_list_2, dx_list, dy_list, de0_list, del0_list = [], [], [], [], [], [], [], []
az_list_1.append(result[:,2])
el_list_1.append(result[:,3])
az_list_2.append(result[:,5])
el_list_2.append(result[:,6])
dx_list.append(result[:,4])
dy_list.append(result[:,7])
de0_list.append(result[:,0])
del0_list.append(result[:,1])

old_kisa = [float(_kisa) for _kisa in open(hosei_path)]
de0 = old_kisa[16]
del0 = old_kisa[17]
corv0 = old_kisa[18]
corp0 = old_kisa[19]
g0 = old_kisa[20]
gg0 = old_kisa[21]


# ### all_offset???                                                                                                            
def calc_dx(az, el, dx, kisa):
    dx_ = []
    for i in range(len(dx)):
        d = kisa[18] * cos(d2r(el[i] + kisa[19])) + kisa[16] - dx[i]
        dx_.append(d)
    return dx_

def calc_dy(az, el, dy, kisa):
    dy_ = []
    for i in range(len(dy)):
        d = kisa[21] * (el[i] ** 2) + kisa[20] * el[i] - kisa[18] * sin(d2r(el[i] + kisa[19])) + kisa[17] - dy[i]
        dy_.append(d)
    return dy_

# ### ???????                                                                                                              
def plot(az_list_1, el_list_1, az_list_2, el_list_2, dx_list, dy_list, calc_flag = True):
    az1 = [e for inner_list in az_list_1 for e in inner_list]
    el1 = [e for inner_list in el_list_1 for e in inner_list]
    az2 = [e for inner_list in az_list_2 for e in inner_list]
    el2 = [e for inner_list in el_list_2 for e in inner_list]
    dx = [e for inner_list in dx_list for e in inner_list]
    dy = [e for inner_list in dy_list for e in inner_list]
    if calc_flag == True:
        dx_ = calc_dx(az1, el1, dx, old_kisa)
        dy_ = calc_dy(az2, el2, dy, old_kisa)
        dx_avg, dy_avg, dx_std, dy_std = numpy.average(dx_), numpy.average(dy_), numpy.std(dx_), numpy.std(dy_)

    else :
        dx_avg, dy_avg, dx_std, dy_std = numpy.average(dx), numpy.average(dy), numpy.std(dx), numpy.std(dy)
    nrow = 3
    ncol = 2
    nax = ncol * nrow
    figsize = (ncol * 4, nrow * 4)

    fig = plt.figure(figsize=figsize)
    fig.subplots_adjust(wspace=0.4, hspace=0.4)
    ax = [fig.add_subplot(nrow, ncol, i+1) for i in range(nax-1)]

    matplotlib.rcParams['savefig.dpi'] = 200
    matplotlib.rcParams['font.size'] = 14

    if calc_flag == True:
        for az_1, el_1, az_2, el_2 in zip(az_list_1, el_list_1, az_list_2, el_list_2):
            ax[0].plot(az_1, dx_, '.', label='diff')
            ax[1].plot(el_1, dx_, '.', label='diff')
            ax[2].plot(az_2, dy_, '.', label='diff')
            ax[3].plot(el_2, dy_, '.', label='diff')
            ax[4].plot(dx_, dy_, '.', label='diff')
    else :
        for az_1, el_1, az_2, el_2, dx, dy in zip(az_list_1, el_list_1, az_list_2, el_list_2, dx_list, dy_list):
            ax[0].plot(az_1, dx, '.', label='diff')
            ax[1].plot(el_1, dx, '.', label='diff')
            ax[2].plot(az_2, dy, '.', label='diff')
            ax[3].plot(el_2, dy, '.', label='diff')
            ax[4].plot(dx, dy, '.', label='diff')

    ax[0].set_xlabel('Az [deg.]')
    ax[0].set_ylabel('dx [arcsec.]')
    ax[0].set_title('Az vs dx')

    ax[1].set_xlabel('El [deg.]')
    ax[1].set_ylabel('dx [arcsec.]')
    ax[1].set_title('El vs dx')

    ax[2].set_xlabel('Az [deg.]')
    ax[2].set_ylabel('dy [arcsec.]')
    ax[2].set_title('Az vs dy')

    ax[3].set_xlabel('El [deg.]')
    ax[3].set_ylabel('dy [arcsec.]')
    ax[3].set_title('El vs dy')

    ax[4].set_xlabel('dx [arcsec.]')
    ax[4].set_ylabel('dy [arcsec.]')
    ax[4].set_title('dx vs dy')

    [_ax.grid() for _ax in ax]

    if len(az_list_1) < 8:
        tbl_loc = 'lower center'
        legend_loc = (1.1, 0.35)
    else:
        tbl_loc = 'bottom'
        legend_loc = (1.1, 0.02)

    tbl = fig.add_subplot(3,2,6)
    col_labels=['average','std dev',]
    row_labels=[' dx ',' dy ', ' unite ']
    tbl_vals=[["{:.2e}".format(dx_avg), "{:.2f}".format(dx_std)],
              ["{:.2e}".format(dy_avg), "{:.2f}".format(dy_std)],
              ["{:.2e}".format(numpy.sqrt(dx_avg**2+dy_avg**2)), "{:.2e}".format(numpy.sqrt(dx_std**2+dy_std**2))]
    ]

    tbl.table(cellText=tbl_vals, rowLabels=row_labels, colLabels=col_labels, loc=tbl_loc)

    tbl.set_axis_off()
    fig.tight_layout()
    #ax[4].legend(labels=file_list, loc=legend_loc)                                                                               
    plt.show()

def plot_3d(az_list_1, el_list_1, az_list_2 ,el_list_2, dx_list, dy_list, calc_flag = True):
    nrow = 1
    ncol = 2
    nax = ncol * nrow
    figsize = (ncol * 8, nrow * 6)
    matplotlib.rcParams['font.size'] = 12

    az1 = [e for inner_list in az_list_1 for e in inner_list]
    el1 = [e for inner_list in el_list_1 for e in inner_list]
    az2 = [e for inner_list in az_list_2 for e in inner_list]
    el2 = [e for inner_list in el_list_2 for e in inner_list]
    dx = [e for inner_list in dx_list for e in inner_list]
    dy = [e for inner_list in dy_list for e in inner_list]
    dx_ = calc_dx(az1, el1, dx, old_kisa)
    dy_ = calc_dy(az2, el2, dy, old_kisa)


    for az_1, el_1, az_2, el_2 in zip(az_list_1, el_list_1, az_list_2, el_list_2):
        if calc_flag == True:
            ax[0].scatter(az_1, el_1, dx_)
            ax[1].scatter(az_2, el_2, dy_)
        else:
            ax[0].scatter(az_1, el_1, dx)
            ax[1].scatter(az_2, el_2, dy)

        ax[0].set_xlabel('Az [deg.]')
        ax[0].set_ylabel('El [deg.]')
        ax[0].set_zlabel('dx [arcsec.]')
        ax[0].set_title('Az vs El vs dx')

        ax[1].set_xlabel('Az [deg.]')
        ax[1].set_ylabel('El [deg.]')
        ax[1].set_zlabel('dy [arcsec.]')
        ax[1].set_title('Az vs El vs dy')

        plt.show()
'''
def plot_offset(kisa1, kisa2, az_list_1, el_list_1, az_list_2, el_list_2, dx_list, dy_list, old = False):
    az1 = [e for inner_list in az_list_1 for e in inner_list]
    el1 = [e for inner_list in el_list_1 for e in inner_list]
    az2 = [e for inner_list in az_list_2 for e in inner_list]
    el2 = [e for inner_list in el_list_2 for e in inner_list]
    dx = [e for inner_list in dx_list for e in inner_list]
    dy = [e for inner_list in dy_list for e in inner_list]
    nrow = 2
    ncol = 2
    nax = ncol * nrow
    figsize = (ncol * 4, nrow * 4)
    fig = plt.figure(figsize=figsize)
    fig.subplots_adjust(wspace=0.4, hspace=0.4)
    ax = [fig.add_subplot(nrow, ncol, i+1) for i in range(nax-1)]
    matplotlib.rcParams['savefig.dpi'] = 200
    matplotlib.rcParams['font.size'] = 14
    if old == True:
        d1_ = []
        d2_ = []
        for i in range(len(az1)):
            d1 = old_kisa[16] - dx[i]
            d2 = old_kisa[17] - dy[i]
            d1_.append(d1)
            d2_.append(d2)
    else:                                                                     
        d1_ = []
        d2_ = []
        for i in range(len(az1)):
            d1 = kisa1[0] - dx[i]
            d2 = kisa2[0] - dy[i]
            d1_.append(d1)
            d2_.append(d2)
    ax[0].plot(az1, d1_, '.', label='diff')
    ax[1].plot(el1, d1_, '.', label='diff')
    ax[2].plot(az2, d2_, '.', label='diff')
    ax[3].plot(el2, d2_, '.', label='diff')

    ax[0].set_xlabel('Az [deg.]')
    ax[0].set_ylabel('de_radio - az_offset')
    ax[0].set_title('Az vs de_radio - az_offset') 

    ax[1].set_xlabel('El [deg.]')
    ax[1].set_ylabel('de_radio - az_offset')
    ax[1].set_title('El vs de_radio - el_offset')

    ax[2].set_xlabel('Az [deg.]')
    ax[2].set_ylabel('del_radio - el_offset')
    ax[2].set_title('Az vs del_radio - el_offset')

    ax[3].set_xlabel('El [deg.]')
    ax[3].set_ylabel('del_radio - el_offset')
    ax[3].set_title('El vs del_radio - el_offset')

    [_ax.grid() for _ax in ax]
    if len(az_list_1) < 8:
        tbl_loc = 'lower center'
        legend_loc = (1.1, 0.35)
        
    else:
        tbl_loc = 'bottom'
        legend_loc = (1.1, 0.02)
    fig.tight_layout()
    plt.show()
'''

plot(az_list_1, el_list_1, az_list_2, el_list_2, dx_list, dy_list)
#plot_offset(old_kisa, old_kisa, az_list_1, el_list_1, az_list_2, el_list_2, dx_list, dy_list, old = True)                        
plot_3d(az_list_1, el_list_1, az_list_2 ,el_list_2, dx_list, dy_list)

# ### ????????
'''
def func(kisa, az_1, el_1, az_2, el_2, dx, dy):
    def model_dx(kisa, az, el, dx):
        dx_ = ([corv0]*len(dx)) * cos(d2r(el + ([corp0]*len(dx)))) + ([de0]*len(dx)) - dx
        dx__ = kisa[1] * cos(d2r(el + kisa[2])) + kisa[0]
        residual = dx__ - dx_
        return residual
    def model_dy(kisa, az, el, dy):
        dy_ = ([gg0]*len(dy)) * el * el + ([g0]*len(dy)) * el - ([corv0]*len(dy)) * sin(d2r(el + ([corp0]*len(dy)))) + ([del0]*le\
n(dy)) - dy
        dy__ = kisa[5] * el * el + kisa[4] * el - kisa[1] * sin(d2r(el + kisa[2])) + kisa[3]                                      
        residual = dy__ - dy_
        return residual
    residual_dx = model_dx(kisa, az_1, el_1, dx)
    residual_dy = model_dy(kisa, az_2, el_2, dy)
    return sqrt(residual_dx ** 2 + residual_dy ** 2)
'''
def model_dx(kisa, az, el, dx): # dx??????                                                                                  
    dx_ = ([corv0]*len(dx)) * cos(d2r(el + ([corp0]*len(dx)))) + ([de0]*len(dx)) - dx
    dx__ = kisa[1] * cos(d2r(el + kisa[2])) + kisa[0]
    residual = dx__ - dx_
    return residual

# ### ?????????????
kisa_az0 = [0.] * 3
kisa_el0 = [0.] * 3
kisa_az0[0] = old_kisa[16] # ????????Az????? kisa[16]
kisa_az0[1] = old_kisa[18] # ????????????     kisa[18]
kisa_az0[2] = old_kisa[19] # ?????????????   kisa[19]
kisa_el0[0] = old_kisa[17] # ????????El????? kisa[17]
#kisa_el0[1] = 1
#kisa_el0[2] = 1
kisa_el0[1] = old_kisa[20] # ???????               kisa[20]
kisa_el0[2] = old_kisa[21] # ???????               kisa[21]

'''
kisa0 = [0.] * 6  # initial params.
kisa0[0] = old_kisa[16]      # ????????Az????? kisa[16]
kisa0[3] = old_kisa[17]      # ????????El????? kisa[17]
kisa0[1] = old_kisa[18]    # ????????????     kisa[18]
kisa0[2] = old_kisa[19]    # ?????????????   kisa[19]
#kisa0[4] = 1
#kisa0[5] = 1
kisa0[4] = old_kisa[20]      # ?????  kisa[20]
kisa0[5] = old_kisa[21]      # ?????  kisa[21]
'''

az_1 = numpy.array([e for inner_list in az_list_1 for e in inner_list])
el_1 = numpy.array([e for inner_list in el_list_1 for e in inner_list])
az_2 = numpy.array([e for inner_list in az_list_2 for e in inner_list])
el_2 = numpy.array([e for inner_list in el_list_2 for e in inner_list])
dx = numpy.array([e for inner_list in dx_list for e in inner_list])
dy = numpy.array([e for inner_list in dy_list for e in inner_list])

kisa_az = scipy.optimize.leastsq(model_dx, kisa_az0, args=(az_1, el_1, dx) ,maxfev=1000000)[0]

def model_dy(kisa, az, el, dy): # dy??????
    dy_ = ([gg0]*len(dy)) * el * el + ([g0]*len(dy)) * el - ([corv0]*len(dy)) * sin(d2r(el + ([corp0]*len(dy)))) + ([del0]*len(dy\
)) - dy
    dy__ = kisa[2] * el * el + kisa[1] * el - kisa_az[1] * sin(d2r(el + kisa_az[2])) + kisa[0]
    residual = dy__ - dy_
    return residual

kisa_el = scipy.optimize.leastsq(model_dy, kisa_el0, args=(az_2, el_2, dy) ,maxfev=1000000)[0]
'''                                                                            
kisa = scipy.optimize.leastsq(func, kisa0, args=(az_1, el_1, az_2, el_2, dx, dy) ,maxfev=1000000)[0] 
'''

def pl_dx(az, el, kisa):
    dx = []
    for i in range(len(az)):
        d = kisa[1] * cos(d2r(el[i] + kisa[2])) + kisa[0]
        dx.append(d)
    return dx

def pl_dy(az, el, kisa):
    dy = []
    for i in range(len(az)):
        d = kisa[2] * (el[i] ** 2) + kisa[1] * el[i] - kisa_az[1] * sin(d2r(el[i] + kisa_az[2])) + kisa[0]
        dy.append(d)
    return dy

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
#_dx = pl_dx(AZ, EL, kisa)
#_dy = pl_dy(AZ, EL, kisa)
_dx = pl_dx(AZ, EL, kisa_az)
_dy = pl_dy(AZ, EL, kisa_el)

dx = [e for inner_list in dx_list for e in inner_list]
dy = [e for inner_list in dy_list for e in inner_list]
dx_ = calc_dx(az_1, el_1, dx, old_kisa)
dy_ = calc_dy(az_2, el_2, dy, old_kisa)

for az_1, el_1, az_2, el_2 in zip(az_list_1, el_list_1, az_list_2, el_list_2):
    ax[0].scatter(az_1, el_1, dx_, label='raw', c='steelblue')
    ax[1].scatter(az_2, el_2, dy_, label='raw', c='steelblue')

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

'''
new_dx_list = [ - numpy.array(dx_) + pl_dx(az, el, kisa)
                for az, el in zip(az_list_1, el_list_1)]
new_dy_list = [ - numpy.array(dy_) + pl_dy(az, el, kisa)
                for az, el in zip(az_list_2, el_list_2)]
'''

new_dx_list = [ - numpy.array(dx_) + pl_dx(az, el, kisa_az)
               for az, el in zip(az_list_1, el_list_1)]
new_dy_list = [ - numpy.array(dy_) + pl_dy(az, el, kisa_el)
               for az, el in zip(az_list_2, el_list_2)]


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
#_dx = len(pl_dx(AZ, EL, kisa)) * [0]
#_dy = len(pl_dy(AZ, EL, kisa)) * [0]
_dx = len(pl_dx(AZ, EL, kisa_az)) * [0]
_dy = len(pl_dy(AZ, EL, kisa_el)) * [0]

for az_1, el_1, az_2, el_2, dx, dy in zip(az_list_1, el_list_1, az_list_2, el_list_2, new_dx_list, new_dy_list):
    ax[0].scatter(az_1, el_1, dx, label='raw', c='steelblue')
    ax[1].scatter(az_2, el_2, dy, label='raw', c='steelblue')

ax[0].plot_wireframe(AZ, EL, _dx, label='model', color='blue', linewidth=0.3)
ax[0].set_xlabel('Az [deg.]')
ax[0].set_ylabel('El [deg.]')
ax[0].set_zlabel('dx [arcsec.]')
ax[0].set_title('Az vs El vs dx')

ax[1].plot_wireframe(AZ, EL, _dy, label='model', color='blue', linewidth=0.3)
ax[1].set_xlabel('Az [deg.]')
ax[1].set_ylabel('El [deg.]')
ax[1].set_zlabel('dy [arcsec.]')

plot(az_list_1, el_list_1, az_list_2, el_list_2, new_dx_list, new_dy_list, calc_flag = False)
#plot_offset(kisa_az, kisa_el, az_list_1, el_list_1, az_list_2, el_list_2, dx_list, dy_list)                                      

new_kisa = [0.] * 24

new_kisa[16] = kisa_az[0] # ????????Az?????
new_kisa[17] = kisa_el[0] # ????????El?????
new_kisa[18] = kisa_az[1] # ????????????
new_kisa[19] = kisa_az[2] # ?????????????
new_kisa[20] = kisa_el[1] # ?????
new_kisa[21] = kisa_el[2] # ?????

new_kisa[16] = kisa[0] # ????????Az?????
new_kisa[17] = kisa[3] # ????????El?????
new_kisa[18] = kisa[1] # ????????????
new_kisa[19] = kisa[2] # ?????????????
new_kisa[20] = kisa[4] # ?????
new_kisa[21] = kisa[5] # ?????

index = [
    'Az dif',
    'El dif',
    'corv',
    'corp',
    'g1',
    'g2',
    ]

df = pandas.DataFrame(index=index)
df['dim.'] = [
    'arcsec.',
    'arcsec.',
    'arcsec.',
    'deg.',
    'const.',
    'const.',
]

ind = [16, 17, 18, 19, 20, 21]
old = [old_kisa[_ind] for _ind in ind]
new = [new_kisa[_ind] for _ind in ind]
diff = [_old - _new for _old, _new in zip(old, new)]
df['old'] = old
df['new'] = new
df['diff'] = diff

print(df)


