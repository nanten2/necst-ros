#!/usr/bin/env python3

#----
import sys
import numpy
import matplotlib.pyplot as plt
from astropy.io import fits
from scipy.optimize import curve_fit


para_init1 = numpy.array([75, -1000, 0.0001])###
para_init2 = numpy.array([-75, 1000, 0.0001])###
#------
def gaussian(x, a, mu, gamma):
    return a * numpy.exp(- gamma * (x - mu) **2) 

#-----
def analysis(file_name, integ_mi=5000, integ_ma=10000):
# open file
    hdu = fits.open(file_name)

# define axis / mask
    mode = hdu[1].data["SOBSMODE"]
    lam = hdu[1].data["LAMDEL"]
    bet = hdu[1].data["BETDEL"]
    subscan = hdu[1].data["SUBSCAN"]

    onmask = mode == "ON"
    hotmask = mode == "HOT"
    offmask = mode == "OFF"
    xmask = (subscan == 1) & onmask
    ymask = (subscan == 2) & onmask


# calc Ta*
    integlist = numpy.sum(hdu[1].data["DATA"][:, int(integ_mi):int(integ_ma)], axis = 1) ##TODO change range

    tmp = []
    HOT = integlist[hotmask]
    for i in range(numpy.sum(hotmask) -1):
        tmp.extend([HOT[i] for j in range(int(len(hotmask)/4))])
    tmp.append(HOT[numpy.sum(hotmask) -1])
    HOTlist = numpy.array(tmp)

    tmp = []
    OFF = integlist[offmask]
    for i in range(numpy.sum(offmask)):
        tmp.extend([OFF[i] for j in range(int(len(offmask)/4))])
    tmp.append(OFF[numpy.sum(offmask) -1])
    OFFlist = numpy.array(tmp)

    ONlist = integlist

    Taslist = (ONlist - OFFlist)/(HOTlist - OFFlist) * 300


# create data for plot
    xscan_Ta = Taslist[xmask]
    xscan_x= lam[xmask]
    xscan_y= bet[xmask]

    yscan_Ta = Taslist[ymask]
    yscan_x= lam[ymask]
    yscan_y= bet[ymask]


# color image
    fig = plt.figure()

    ax = fig.add_subplot(1,1,1, aspect = 1)

    im = ax.scatter(xscan_x, xscan_y, c = xscan_Ta, vmin = 0, vmax = 280)
    ax.scatter(yscan_x, yscan_y, c = yscan_Ta, vmin = 0, vmax = 280)

    fig.colorbar(im)


# Differential function
    xscan_tmp = numpy.roll(xscan_Ta,1)
    xscan_tmp[0] = 0
    yscan_tmp = numpy.roll(yscan_Ta, 1)
    yscan_tmp[0] = 0

    xscan_dif = xscan_Ta - xscan_tmp
    yscan_dif = yscan_Ta - yscan_tmp


# Gaussian Fitting function add errorbar
    try:
        x_az = numpy.linspace(xscan_x[0], xscan_x[-1], 2000)

# dAz fitting
        popt_az1, pcov_az1 = curve_fit(gaussian, xscan_x[:21], xscan_dif[:21], p0 = para_init1)
        error_az1 = numpy.sqrt(numpy.diag(pcov_az1))
#print("error",error_az1)
        popt_az2, pcov_az2 = curve_fit(gaussian, xscan_x[21:], xscan_dif[21:], p0 = para_init2)
        error_az2 = numpy.sqrt(numpy.diag(pcov_az2))
#print("error",error_az2)

        gaus_az1 = gaussian(x_az[:1000], popt_az1[0], popt_az1[1], popt_az1[2])
        gaus_az2 = gaussian(x_az[1000:], popt_az2[0], popt_az2[1], popt_az2[2])

        x_el = numpy.linspace(yscan_y[0], yscan_y[-1], 2000)
# El fitting
        popt_el1, pcov_el1 = curve_fit(gaussian, yscan_y[:21], yscan_dif[:21], p0 = para_init1)
        error_el1 = numpy.sqrt(numpy.diag(pcov_el1))
        popt_el2, pcov_el2 = curve_fit(gaussian, yscan_y[21:], yscan_dif[21:], p0 = para_init2)
        error_el2 = numpy.sqrt(numpy.diag(pcov_el2))

        gaus_el1 = gaussian(x_el[:1000], popt_el1[0], popt_el1[1], popt_el1[2])
        gaus_el2 = gaussian(x_el[1000:], popt_el2[0], popt_el2[1], popt_el2[2])


# dAz dEl
        dAz_mi = popt_az1[1]
        dAz_pu = popt_az2[1]
        dEl_mi = popt_el1[1]
        dEl_pu = popt_el2[1]
        dAz = (dAz_mi + dAz_pu)/2
        dEl = (dEl_mi + dEl_pu)/2


# Dif plot
        fig2 = plt.figure(figsize = (18, 8))

        axlist = [fig2.add_subplot(3,2,i+1) for i in range(6)]

        axlist[0].plot(xscan_x, xscan_Ta, "o")
        axlist[0].vlines(dAz_mi, 0, 300, linestyle = "dashed")
        axlist[0].vlines(dAz_pu, 0, 300, linestyle = "dashed")
        axlist[0].vlines(dAz, 0, 300)
        axlist[0].set_ylabel("Ta* [K]")

        axlist[1].plot(yscan_y, yscan_Ta, "o")
        axlist[1].vlines(dEl_mi, 0, 300, linestyle = "dashed")
        axlist[1].vlines(dEl_pu, 0, 300, linestyle = "dashed")
        axlist[1].vlines(dEl, 0, 300)
        axlist[1].set_ylabel("Ta* [K]")

        axlist[2].plot(xscan_x, xscan_dif, "o")
        axlist[2].plot(x_az[:1000], gaus_az1, color="k")
        axlist[2].plot(x_az[1000:], gaus_az2, color="k")
        axlist[2].set_xlabel("dAz [arcsec]")
        axlist[2].set_ylabel("differential [K]")

        axlist[3].plot(yscan_y, yscan_dif, "o")
        axlist[3].plot(x_el[:1000], gaus_el1, color="k")
        axlist[3].plot(x_el[1000:], gaus_el2, color="k")
        axlist[3].set_xlabel("dEl [arcsec]")
        axlist[3].set_ylabel("differential [K]")

        axlist[4].set_visible(False)
        axlist[5].set_visible(False)

        plt.axes([0.45,0.28, 0.25, 0.2])
        plt.axis("off")
        plt.text(0,0,"dAz = {}".format(round(dAz, 2)) + "              dEl = {}".format(round(dEl, 2)) + "   (arcsec)", fontsize = 16)
        plt.text(0, -0.3, "DATA PATH :   {}".format(file_name), fontsize=10)

        [a.grid() for a in axlist]

    except Exception as e:
        print("\033[31m[ERROR OCCURRED]\033[0m\n", e)
        fig2 = plt.figure(figsize = (18, 8))

        axlist = [fig2.add_subplot(3,2,i+1) for i in range(6)]

        axlist[0].plot(xscan_x, xscan_Ta, "o")
        axlist[0].set_ylabel("Ta* [K]")

        axlist[1].plot(yscan_y, yscan_Ta, "o")
        axlist[1].set_ylabel("Ta* [K]")

        axlist[2].plot(xscan_x, xscan_dif, "o")
        axlist[2].set_xlabel("dAz [arcsec]")
        axlist[2].set_ylabel("differential [K]")

        axlist[3].plot(yscan_y, yscan_dif, "o")
        axlist[3].set_xlabel("dEl [arcsec]")
        axlist[3].set_ylabel("differential [K]")

        axlist[4].set_visible(False)
        axlist[5].set_visible(False)

        plt.axes([0.45,0.28, 0.25, 0.2])
        plt.axis("off")
        plt.text(0,0, "ERROR OCCURRED", fontsize = 16)
        plt.text(0, -0.3, "DATA PATH :   {}".format(file_name), fontsize=10)

            
        [a.grid() for a in axlist]
    
    finally:
        plt.show()

    return

if __name__ =="__main__":
    args = sys.argv
    if len(args) < 2:
        print("You must specify data_file")
        sys.exit()
    file_name = args[1]

# option
# integration range
    integ_mi = int(5000)
    integ_ma = int(10000)

# specify option
    if len(args) == 4:
        if args[2] != "DEF":
            integ_mi = int(args[2])
        if args[3] != "DEF":
            integ_ma = int(args[3])
    else: pass

    analysis(file_name, integ_mi, integ_ma)
