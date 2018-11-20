#!/usr/bin/env python3

#----
import sys
import numpy
import matplotlib.pyplot as plt
from astropy.io import fits
from scipy.optimize import curve_fit

#-------
def gaussian(x, a, mu, gamma):
    return a * numpy.exp(- gamma * (x - mu) **2) 

# for gaussian fitting
para_init = numpy.array([10, 0.1, 0.0001])
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


# get index
    ind_hot = numpy.where(hotmask == True)
    ind_list_hot = [ind_hot[0][i] for i in range(len(ind_hot[0]))]

    ind_off = numpy.where(offmask == True)
    ind_list_off = [ind_off[0][i] for i in range(len(ind_off[0]))]

# calc Ta*
    data = hdu[1].data["DATA"]

    HOTlist = numpy.zeros((len(hotmask),16384))
    for i in range(len(ind_list_hot)-1):
        HOTlist[ind_list_hot[i]:ind_list_hot[i+1]] = data[ind_list_hot[i]]
    HOTlist[ind_list_hot[-1]:(len(hotmask))] = data[ind_list_hot[-1]]

    OFFlist = numpy.zeros((len(offmask),16384))
    for i in range(len(ind_list_off)-1):
        OFFlist[ind_list_off[i]:ind_list_off[i+1]] = data[ind_list_off[i]]
    OFFlist[ind_list_off[-1]:(len(offmask))] = data[ind_list_off[-1]]

    ONlist = numpy.array(data)

    Taslist = (ONlist - OFFlist)/(HOTlist - OFFlist) * 300


# create data for plot
    if numpy.sum(subscan):
        xscan_Ta = Taslist[xmask]
        xscan_x= lam[xmask]
        xscan_y= bet[xmask]

        yscan_Ta = Taslist[ymask]
        yscan_x= lam[ymask]
        yscan_y= bet[ymask]
        
    else:
        xscan_Ta = Taslist[onmask][:5]
        xscan_x= lam[onmask][:5]
        xscan_y= bet[onmask][:5]

        yscan_Ta = Taslist[onmask][5:]
        yscan_x= lam[onmask][5:]
        yscan_y= bet[onmask][5:]


# TA* integration
    xscan_integ = numpy.sum(xscan_Ta[:, int(integ_mi):int(integ_ma)], axis=1)
    yscan_integ = numpy.sum(yscan_Ta[:, int(integ_mi):int(integ_ma)], axis=1)


# Gaussian Fitting function add errorbar
# Az fitting
    try:
        popt_az, pcov_az = curve_fit(gaussian, xscan_x, xscan_integ, p0 = para_init, maxfev=10000)
        error_az = numpy.sqrt(numpy.diag(pcov_az))

        x_g = numpy.linspace(xscan_x[0], xscan_x[-1], 1001)
        gaus_az = gaussian(x_g, popt_az[0], popt_az[1], popt_az[2])

# El fitting
        popt_el, pcov_el = curve_fit(gaussian, yscan_y, yscan_integ, p0 = para_init, maxfev=10000)
        error_el = numpy.sqrt(numpy.diag(pcov_el))

        gaus_el = gaussian(x_g, popt_el[0], popt_el[1], popt_el[2])


# dAz dEl
        dAz = popt_az[1]
        dEl = popt_el[1]
        hpbw_az =  1/numpy.sqrt(2*popt_az[2]) *2.35
        hpbw_el = 1/numpy.sqrt(2*popt_el[2]) *2.35


# plot
        fig = plt.figure(figsize = (15, 5))

        axlist = [fig.add_subplot(2,2,i+1) for i in range(4)]

        axlist[0].plot(xscan_x, xscan_integ, "o")
        axlist[0].errorbar(xscan_x, xscan_integ, yerr = error_az[0], fmt = "b+")
        axlist[0].plot(x_g, gaus_az)
        axlist[0].set_xlabel("dAz [arcsec]")
        axlist[0].set_ylabel("Ta* [K]")

        axlist[1].plot(yscan_y, yscan_integ, "o")
        axlist[1].errorbar(yscan_y, yscan_integ, yerr = error_el[0], fmt = "b+")
        axlist[1].plot(x_g, gaus_el)
        axlist[1].set_xlabel("dEl [arcsec]")
        axlist[1].set_ylabel("Ta* [K]")

        axlist[2].set_visible(False)
        axlist[3].set_visible(False)

        plt.axes([0.55,0.25, 0.25, 0.2])
        plt.axis("off")
        plt.text(0,0,"dAz = {}".format(round(dAz, 2)) + "              dEl = {}".format(round(dEl, 2)) + "   (arcsec)", fontsize = 10)
        plt.text(0,-0.5,"HPBW_AZ = {}".format(round(hpbw_az, 2)) + "      HPBW_EL = {}".format(round(hpbw_el, 2)), fontsize = 10)
        plt.text(0, -1.0, "DATA PATH :   {}".format(file_name), fontsize=6)

        [a.grid() for a in axlist]
    
    except Exception as e:
        print("\033[31m[ERROR OCCURRED]\033[0m\n", e)
        
        fig = plt.figure(figsize = (20,15))

        axlist = [fig.add_subplot(5,5,i+1) for i in range(25)]

        axlist[2].plot(yscan_Ta[0])
        axlist[2].set_xlabel("")
        axlist[2].set_ylabel("Count")

        axlist[7].plot(yscan_Ta[1])
        axlist[7].set_xlabel("")
        axlist[7].set_ylabel("Count")

        axlist[10].plot(xscan_Ta[0])
        axlist[10].set_xlabel("")
        axlist[10].set_ylabel("Count")

        axlist[11].plot(xscan_Ta[1])
        axlist[11].set_xlabel("")
        axlist[11].set_ylabel("Count")

        # axlist[12].plot(xscan_Ta[2])
        axlist[12].plot(yscan_Ta[2])
        axlist[12].set_xlabel("")
        axlist[12].set_ylabel("Count")

        axlist[13].plot(xscan_Ta[3])
        axlist[13].set_xlabel("")
        axlist[13].set_ylabel("Count")

        axlist[14].plot(xscan_Ta[4])
        axlist[14].set_xlabel("")
        axlist[14].set_ylabel("Count")

        axlist[17].plot(yscan_Ta[3])
        axlist[17].set_xlabel("")
        axlist[17].set_ylabel("Count")

        axlist[22].plot(yscan_Ta[4])
        axlist[22].set_xlabel("")
        axlist[22].set_ylabel("Count")

        axlist[0].set_visible(False)
        axlist[1].set_visible(False)
        axlist[3].set_visible(False)
        axlist[4].set_visible(False)
        axlist[5].set_visible(False)
        axlist[6].set_visible(False)
        axlist[8].set_visible(False)
        axlist[9].set_visible(False)
        axlist[15].set_visible(False)
        axlist[16].set_visible(False)
        axlist[18].set_visible(False)
        axlist[19].set_visible(False)
        axlist[20].set_visible(False)
        axlist[21].set_visible(False)
        axlist[23].set_visible(False)
        axlist[24].set_visible(False)

        plt.axes([0.55,0.25, 0.25, 0.2])
        plt.axis("off")
        plt.text(0.5,-0.5, "ERROR OCCURRED", fontsize = 10)
        plt.text(0.3, -1.0, "DATA PATH :   {}".format(file_name), fontsize=6)

        [a.grid() for a in axlist]

    finally:
        plt.show()

    return

if __name__ == "__main__":
    args = sys.argv
    if len(args) < 2:
        print("You must specify data_file")
        sys.exit()

    file_name = args[1]
# option
# integration range
    integ_mi = int(8000)
    integ_ma = int(9000)

# specify option
    if len(args) == 4:
        if args[2] != "DEF":
            integ_mi = int(args[2])
        if args[3] != "DEF":
            integ_ma = int(args[3])
    else: pass

    analysis(file_name, integ_mi, integ_ma)
