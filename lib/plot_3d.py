#!/usr/bin/env python3

import numpy
import matplotlib
import matplotlib.pyplot as plt

def plot(az_list, el_list, dx_list, dy_list):

    nrow = 1
    ncol = 2
    nax = ncol * nrow
    figsize = (ncol * 8, nrow * 6)
    matplotlib.rcParams['font.size'] = 12

    fig = plt.figure(figsize=figsize)
    ax = [fig.add_subplot(nrow, ncol, i+1, projection='3d') for i in range(nax)]

    for az, el, dx, dy in zip(az_list, el_list, dx_list, dy_list):
        ax[0].scatter(az, el, dx)
        ax[1].scatter(az, el, dy)

        ax[0].set_xlabel('Az [deg.]')
        ax[0].set_ylabel('El [deg.]')
        ax[0].set_zlabel('dx [arcsec.]')
        ax[0].set_title('Az vs El vs dx')    

        ax[1].set_xlabel('Az [deg.]')
        ax[1].set_ylabel('El [deg.]')
        ax[1].set_zlabel('dy [arcsec.]')
        ax[1].set_title('Az vs El vs dy')

        plt.show()
