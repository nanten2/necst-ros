#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.optimize import curve_fit


def process2forfit(dir):
        d = np.loadtxt(os.path.join(dir, 'process.log'))
        data = np.array([d[:,14],d[:,15],(d[:,10]-350)*0.9267,(d[:,9]-240)*0.8392,
        d[:,16],d[:,17]])
        result = data.T
        np.savetxt(os.path.join(dir, 'for_fit.log'), result, delimiter=' ')
        return


def process_static(dir_list, *, clip_sigma=None, clip_const=None):
    ind_az = 14
    ind_el = 15
    ind_dx = 10
    ind_dy = 9

    rawdata = np.concatenate([np.loadtxt(os.path.join(_dir, 'process.log')) for _dir in dir_list])

    rawdata[:,ind_dx]=(rawdata[:,ind_dx]-109)*1.18
    rawdata[:,ind_dy]=(rawdata[:,ind_dy]-210)*1.18

    #rawdx_avg = np.average(rawdata[:,ind_dx])
    #rawdy_avg = np.average(rawdata[:,ind_dy])
    rawdx_std = np.std(rawdata[:,ind_dx])
    rawdy_std = np.std(rawdata[:,ind_dy])
    rawdx_med = np.median(rawdata[:,ind_dx])
    rawdy_med = np.median(rawdata[:,ind_dy])
    #print(d)

    if clip_sigma != None:
        x_clip, y_clip = clip_sigma
        d=rawdata[(abs(rawdata[:,ind_dx] -rawdx_med) < x_clip * rawdx_std) & (abs(rawdata[:,ind_dy] -rawdy_med) < y_clip * rawdy_std)]
    elif clip_const != None:
        x_clip, y_clip = clip_const
        d=rawdata[(abs(rawdata[:,ind_dx] -rawdx_med) < x_clip) & (abs(rawdata[:,ind_dy] -rawdy_med) < y_clip)]
    else:
        d = rawdata

    dx_avg = np.average(d[:,ind_dx])
    dy_avg = np.average(d[:,ind_dy])
    dx_std = np.std(d[:,ind_dx])
    dy_std = np.std(d[:,ind_dy])
    dx_med = np.median(d[:,ind_dx])
    dy_med = np.median(d[:,ind_dy])

    #print('dx average : ', dx_avg)
    #print('dy average : ', dy_avg)
    #print('dx dispersion : ', dx_var)
    #print('dy dispersion : ', dy_var)

    return dx_avg, dy_avg, dx_std, dy_std, dx_med, dy_med


def opt_plot(dir_list, *, clip_sigma=(3.,3.), savefig=True, figname=None, interactive=False):

    ind_az = 14
    ind_el = 15
    ind_dx = 10
    ind_dy = 9

    dx_avg, dy_avg, dx_std, dy_std, *_ = process_static(dir_list, clip_sigma=clip_sigma)

    #files = multifiles(dirname)
    d_list = [np.loadtxt(os.path.join(_dir, 'process.log')) for _dir in dir_list]
    for _d in d_list:
        _d[:,ind_dx]=(_d[:,ind_dx]-109)*1.18
        _d[:,ind_dy]=(_d[:,ind_dy]-210)*1.18
        _dx=(_d[:,ind_dx]*np.cos(np.radians(-1.76))-_d[:,ind_dy]*np.sin(np.radians(-1.76)))
        _dy=(_d[:,inf_dx]*np.sin(np.radians(-1.76))+_d[:,ind_dy]*np.cos(np.radians(-1.76)))
        _d = _d[(abs(_dx -dx_avg) < dx_std * clip_sigma[0]) & (abs(_dy -dy_avg) < dy_std * clip_sigma[1])]

    axes = np.array([[ind_az, ind_dx, 'Az', 'dx'],
    [ind_az, ind_dy,'Az', 'dy'],
    [ind_el, ind_dx, 'El', 'dx'],
    [ind_el, ind_dy, 'El', 'dy'],
    [ind_dx, ind_dy, 'dx', 'dy']])

    fig = plt.figure(figsize=(6.5,9))
    ax = [fig.add_subplot(3, 2, i) for i in range(1, 6)]
    [[_a.scatter(_d[:,int(_xind)], _d[:,int(_yind)], marker='.') for _a, _xind, _yind in zip(ax, axes[:,0], axes[:,1])] for _d in d_list]
    [_a.grid(True) for _a in ax]
    [_a.set_title(_x+' - '+_y) for _a, _x, _y in zip(ax, axes[:,2], axes[:,3])]
    [_a.set_xlabel(_x+' (deg)') for _a, _x in zip(ax, axes[:,2])]
    ax[4].set_xlabel('dx (arcsec)')
    [_a.set_ylabel(_y+' (arcsec)') for _a, _y in zip(ax, axes[:,3])]

    if len(dir_list) < 8:
        tbl_loc = 'lower center'
        legend_loc = (1.1, 0.35)
    else:
        tbl_loc = 'bottom'
        legend_loc = (1.1, 0.02)

    tbl = fig.add_subplot(3,2,6)

    col_labels=['average','std dev']
    row_labels=[' dx ',' dy ']
    tbl_vals=[["{:.2e}".format(dx_avg), "{:.2f}".format(dx_std)],
["{:.2e}".format(dy_avg), "{:.2f}".format(dy_std)]]

    tbl.table(cellText=tbl_vals,
    rowLabels=row_labels,
    colLabels=col_labels,
    loc=tbl_loc)

    tbl.set_axis_off()
    fig.tight_layout()
    ax[4].legend(labels=dir_list, loc=legend_loc)

    if savefig == True:
        if figname == None:
            figname = 'plot_tmp.png'
        fig.savefig(figname)

    if interactive == True:
        plt.show()

    return


def fdx(altaz, dAz, de, chi, omega, eps):
    Az, El = altaz
    El = np.deg2rad(El)
    dx = chi * np.sin(np.deg2rad(omega - Az)) * np.sin(El) + eps * np.sin(El) + dAz * np.cos(El) + de
    return dx

def fdy(altaz, dEl, chi, omega, g1):
    Az, El = altaz
    dy = -chi * np.cos(np.deg2rad(omega - Az)) + dEl + g1 * El
    return dy


def opt_fit(dir_list, *, hosei_path='hosei_opt.txt', output_dir=None, savefig=True, figname=None, interactive=False):
    if output_dir == None:
        output_dir = '.'
    else:
        if os.path.isdir(output_dir) == False:
            os.makedirs(output_dir)

    ind_az = 0
    ind_el = 1
    ind_dx = 2
    ind_dy = 3

    dx_avg, dy_avg, dx_std, dy_std, *_= process_static(dir_list, clip_sigma=(3.,3.))
    d_list_noclip = [np.loadtxt(os.path.dirname(_dir + '/')+'/for_fit.log') for _dir in dir_list]
    d_list = [_d[(abs(_d[:,ind_dx] -dx_avg) < dx_std * 3.) & (abs(_d[:,ind_dy] -dy_avg) < dy_std * 3.)] for _d in d_list_noclip]

    hosei = np.loadtxt(hosei_path)
    dAz = hosei[0]
    de = hosei[1]
    chi_x = hosei[2]
    omega_x = hosei[3]
    eps = hosei[4]
    chi_y = hosei[7]
    omega_y = hosei[8]
    g1 = hosei[11]
    dEl = hosei[15]

    back_list = [_d[:,(ind_dx, ind_dy)] +  np.array([fdx(_d[:,(ind_az, ind_el)].T, dAz, de, chi_x, omega_x, eps), fdy(_d[:,(ind_az, ind_el)].T, dEl, chi_y, omega_y, g1)]).T for _d in d_list]
    back = np.concatenate(back_list)

    p_opt_dx, coval_dx = curve_fit(fdx, np.concatenate(d_list)[:,(ind_az,ind_el)].T, back[:,0], p0=np.array([dAz, de, chi_x, omega_x, eps]))
    p_opt_dy, coval_dy = curve_fit(fdy, np.concatenate(d_list)[:,(ind_az,ind_el)].T, back[:,1], p0=np.array([dEl, chi_y, omega_y, g1]))

    dAz_n, de_n, chi_xn, omega_xn, eps_n = p_opt_dx
    dEl_n, chi_yn, omega_yn, g1_n = p_opt_dy

    res_list = [_b - np.array([fdx(_d[:,(ind_az, ind_el)].T, dAz_n, de_n, chi_xn, omega_xn, eps_n), fdy(_d[:,(ind_az, ind_el)].T, dEl_n, chi_yn, omega_yn, g1_n)]).T for _b, _d in zip(back_list, d_list)]
    res = np.concatenate(res_list)
    res_dx_avg, res_dy_avg = np.average(res, axis=0)
    res_dx_std, res_dy_std = np.std(res, axis=0)
    ind_resdx = 0
    ind_resdy = 1

    fig = plt.figure(figsize=(6.5,9))
    axes = np.array([[ind_az, ind_resdx, 'Az', 'dx'],
    [ind_az, ind_resdy, 'Az', 'dy'],
    [ind_el, ind_resdx, 'El', 'dx'],
    [ind_el, ind_resdy, 'El', 'dy'],
    [ind_resdx, ind_resdy, 'dx', 'dy']])
    ax = [fig.add_subplot(3, 2, i) for i in range(1, 6)]
    [[_a.scatter(_d[:,int(_xind)], _r[:,int(_yind)], marker='.') for _a, _xind, _yind in zip(ax[0:4], axes[:4,0], axes[:4,1])] for _d, _r in zip(d_list, res_list)]
    [ax[4].scatter(_r[:,0], _r[:,1], marker='.') for _r in res_list]
    [_a.grid(True) for _a in ax]
    [_a.set_title(_x+' - '+_y) for _a, _x, _y in zip(ax, axes[:,2], axes[:,3])]
    [_a.set_xlabel(_x+' (deg)') for _a, _x in zip(ax, axes[:,2])]
    ax[4].set_xlabel('dx (arcsec)')
    [_a.set_ylabel(_y+' (arcsec)') for _a, _y in zip(ax, axes[:,3])]

    if len(dir_list) < 8:
        tbl_loc = 'lower center'
        legend_loc = (1.1, 0.35)
    else:
        tbl_loc = 'bottom'
        legend_loc = (1.1, 0.02)

    tbl = fig.add_subplot(3,2,6)
    col_labels=['average','std dev']
    row_labels=[' dx ',' dy ']
    tbl_vals=[["{:.2e}".format(res_dx_avg), "{:.2f}".format(res_dx_std)],
    ["{:.2e}".format(res_dy_avg), "{:.2f}".format(res_dy_std)]]
    tbl.table(cellText=tbl_vals,rowLabels=row_labels,colLabels=col_labels,loc=tbl_loc)

    fig.tight_layout()
    tbl.set_axis_off()
    ax[4].legend(labels=dir_list, loc=legend_loc)

    hosei_op = hosei.copy()
    hosei_op[0] = dAz_n
    hosei_op[1] = de_n
    hosei_op[2] = chi_xn
    hosei_op[3] = omega_xn
    hosei_op[4] = eps_n
    hosei_op[7] = chi_yn
    hosei_op[8] = omega_yn
    hosei_op[11] = g1_n
    hosei_op[15] = dEl_n
    np.savetxt(os.path.join(output_dir, 'new_hosei_opt.txt'), hosei_op.T)

    hosei_uct = np.zeros(16)
    hosei_uct[0:5] = np.sqrt(np.diag(coval_dx))
    hosei_uct[[7, 8, 11, 15]] = np.sqrt(np.diag(coval_dy))
    param_names = ['daz', 'de', 'chi_az', 'omega_az', 'eps', 'chi2_az', 'omega2_az', 'chi_el', 'omega_el', 'chi2_el', 'omega2_el', 'g', 'gg', 'ggg', 'gggg', 'del']

    hosei_print = ['| {} | {} | {} | {} | {:.3%} |'.format(_nm, _ih, _oh, _oh - _ih, _uct) for _nm, _ih, _oh, _uct in zip(param_names, hosei[:16], hosei_op[:16], hosei_uct)]
    f = open(os.path.join(output_dir, 'fitting_result.txt'), mode='a')
    f.write("-" * 32 + "\n")
    f.write("directories of data :\t{}\n".format(',\t'.join(dir_list)))
    f.write("number of stars :\t{}\n".format(',\t'.join([str(len(_d)) for _d in d_list])))
    f.write("number of cliped stars :\t{}\n".format(',\t'.join([str(len(_dc) - len(_d)) for _dc, _d in zip(d_list_noclip, d_list)])))
    f.write("total number of stars : \t{}\n".format(sum([(len(_d)) for _d in d_list])))
    f.write("parameter, new value, previous value, delta, uncertainty")
    f.write('\n'.join(hosei_print) + '\n')
    f.close()

    if savefig == True:
        if figname == None:
            figname = 'residual_plot.png'
        fig.savefig(os.path.join(output_dir, figname))

    if interactive == True:
        plt.show()

    return


if __name__ == '__main__':
    import sys

    args = sys.argv
    processing = args[1]
    dir_list = args[2:]

    if processing == 'p':
        opt_plot(dir_list,savefig=True, interactive=False)

    elif processing == 'f':
        opt_fit(dir_list)

    elif processing == 'c':
        [process2forfit(_dir) for _d in dir_list]

    elif processing == 'a':
        opt_plot(dir_list,savefig=True, interactive=False)
        [process2forfit(_d) for _d in dir_list]
        opt_fit(dir_list)

    else:
        print("Assign 'plot' or 'fit' or 'convert' or 'all'")
