#!/usr/bin/env python3

import numpy
import opt_analy
import matplotlib
import matplotlib.pyplot as plt

def plot(az_list, el_list, dx_list, dy_list, file_list, raw=True):

    if raw == True:
        dx_avg, dy_avg, dx_std, dy_std, *_ = opt_analy.process_static(file_list, clip_sigma=(3.,3.))
    else:
        az = [e for inner_list in az_list for e in inner_list]
        el = [e for inner_list in el_list for e in inner_list]
        dx = [e for inner_list in dx_list for e in inner_list]
        dy = [e for inner_list in dy_list for e in inner_list]
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

    for az, el, dx, dy in zip(az_list, el_list, dx_list, dy_list):
        ax[0].plot(az, dx, '.', label='diff')
        ax[1].plot(el, dx, '.', label='diff')
        ax[2].plot(az, dy, '.', label='diff')
        ax[3].plot(el, dy, '.', label='diff')
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

    if len(file_list) < 8:
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
    ax[4].legend(labels=file_list, loc=legend_loc)

    plt.show()
