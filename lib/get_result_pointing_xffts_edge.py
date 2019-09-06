#!/usr/bin/env python3
import glob
###
key_word = '20181114'
###

dir_list = glob.glob('/home/amigos/data/ps_edge_pointing/*{}*pointing'.format(key_word))
print(dir_list)

###pointing line
#----
import sys
sys.path.append('/home/amigos/ros/src/necst/lib')
import n2df
import numpy
import matplotlib.pyplot as plt
#from astropy.io import fits
from scipy.optimize import curve_fit
#-----

def f(x, a, b, c):
    return a*x**2 + b*x + c

def gaussian(x, a, mu, gamma):
    return a * numpy.exp(- gamma * (x - mu) **2)

para_init = numpy.array([25000., 0.1, 0.0001])

def calc_integdata(IF, data_list, mode_list, lam, bet, scan_list, integ_mi, integ_ma):

    data_list = data_list[IF-1]
    mode_list = mode_list[IF-1]
    lam = lam[IF-1]
    bet = bet[IF-1]
    scan_list = scan_list[IF-1]
    
    xmask = []
    ymask = []
    hotmask = []
    offmask = []
    for i in range(len(mode_list)):  
        if mode_list[i] == 'HOT':
            xmask.append(0)
            ymask.append(0)
            hotmask.append(1)
            offmask.append(0)
        elif mode_list[i] == 'OFF':
            xmask.append(0)
            ymask.append(0)
            hotmask.append(0)
            offmask.append(1)
        elif scan_list[i] == 1 and mode_list[i] == 'ON':
            xmask.append(1)
            ymask.append(0)
            hotmask.append(0)
            offmask.append(0)
        elif scan_list[i] == 2 and mode_list[i] == 'ON':
            xmask.append(0)
            ymask.append(1)
            hotmask.append(0)
            offmask.append(0)
    
# calc Ta*
    integlist = []
    for i in range(len(data_list)):
        l = data_list[i]
        integlist.append(numpy.sum(l[int(integ_mi):int(integ_ma)]))

    tmp = []
    HOT = []
    for i in range(len(hotmask)):
        if hotmask[i] == 1:
            HOT.append(integlist[i])
        else:
            pass
    for i in range(numpy.sum(hotmask) -1):
        t = []
        for j in range(int(len(hotmask)/4)):
            t.append(HOT[i])
        tmp.extend(t)
    tmp.append(HOT[numpy.sum(hotmask) -1])
    HOTlist = numpy.array(tmp)
    
    tmp = []
    OFF = []
    for i in range(len(offmask)):
        if offmask[i] == 1:
            OFF.append(integlist[i])
        else:
            pass
    for i in range(numpy.sum(offmask)):
        t = []
        for j in range(int(len(offmask)/4)):
            t.append(OFF[i])
        tmp.extend(t)
    tmp.append(OFF[numpy.sum(offmask) -1])
    OFFlist = numpy.array(tmp)
    
    ONlist = numpy.array(integlist)
    
    Taslist = (ONlist - OFFlist)/(HOTlist - OFFlist) * 300
    
    
# create data for plot
    xscan_Ta = []
    xscan_x = []
    xscan_y = []
    
    yscan_Ta = []
    yscan_x = []
    yscan_y = []

    for i in range(len(xmask)):
        if xmask[i] == 1:
            xscan_Ta.append(Taslist[i])
            xscan_x.append(lam[i])
            xscan_y.append(bet[i])
        else:
            pass

    for i in range(len(ymask)):
        if ymask[i] == 1:
            yscan_Ta.append(Taslist[i])
            yscan_x.append(lam[i])
            yscan_y.append(bet[i])
        else:
            pass

    return xscan_Ta, xscan_x, xscan_y, yscan_Ta, yscan_x, yscan_y
#-----
def analysis(directory, file_name, integ_mi=5000, integ_ma=10000, object='sun'):
# open file

    n = n2df.Read(file_name)    
    _n = n.read_all()
    d = []
    for i in range(25):
        _d = []
        for j in range(len(_n)):
            _d.append(_n[j][i])
        d.append(_d)
    
    
# define axis / mask /planet

    time = d[0]
    mode = d[21]
    mode = list(map(lambda x:x.decode() ,mode))
    subscan = d[22]
    _lam = d[23]
    _bet = d[24]
    
# get integdata / mask
    data_list = []
    mode_list = []
    scan_list = []
    lam = []
    bet = []

    print(len(d))
    
    for h in range(20):
        d_ = d[h+1]
        d_list = []
        m_list = []
        s_list = []
        la_list = []
        be_list = []
        tmp = numpy.zeros(32768)
        for i in range(len(d_)):
            if subscan[i] == 1 and mode[i] == 'ON':
                #tmp += pickle.loads(d_[i])
                tmp += d_[i]
                if subscan[i+1] != subscan[i] or i == len(d_)-1 or _lam[i+1] != _lam[i]:
                    d_list.append(tmp)
                    m_list.append('ON')
                    la_list.append(_lam[i])
                    be_list.append(_bet[i])
                    s_list.append(1)
                    tmp = numpy.zeros(32768)
                else:
                    pass
            elif subscan[i] == 2 and mode[i] == 'ON':
                #tmp += pickle.loads(d_[i])
                tmp += d_[i]
                if subscan[i+1] != subscan[i] or i == len(d_)-1 or _lam[i+1] != _lam[i]:
                    d_list.append(tmp)
                    m_list.append('ON')
                    la_list.append(_lam[i])
                    be_list.append(_bet[i])
                    s_list.append(1)
                    tmp = numpy.zeros(32768)
                else:
                    pass
            elif subscan[i] == 3 and mode[i] == 'ON':
                #tmp += pickle.loads(d_[i])
                tmp += d_[i]
                if subscan[i+1] != subscan[i] or i == len(d_)-1 or _bet[i+1] != _bet[i]:
                    d_list.append(tmp)
                    m_list.append('ON')
                    la_list.append(_lam[i])
                    be_list.append(_bet[i])
                    s_list.append(2)
                    tmp = numpy.zeros(32768)
                else:
                    pass
            elif subscan[i] == 4 and mode[i] == 'ON':
                #tmp += pickle.loads(d_[i])
                tmp += d_[i]
                if subscan[i+1] != subscan[i] or i == len(d_)-1 or _bet[i+1] != _bet[i]:
                    d_list.append(tmp)
                    m_list.append('ON')
                    la_list.append(_lam[i])
                    be_list.append(_bet[i])
                    s_list.append(2)
                    tmp = numpy.zeros(32768)
                else:
                    pass
            elif mode[i] == 'OFF':
                #tmp += pickle.loads(d_[i])
                tmp += d_[i]
                if i == len(d_)-1:
                    d_list.append(tmp)
                    m_list.append('OFF')
                    la_list.append(0)
                    be_list.append(0)
                    if subscan[i] == 1 or 2:
                        s_list.append(1)
                    else:
                        s_list.append(2)
                    tmp = numpy.zeros(32768)
                else: 
                    if mode[i+1] != 'OFF':
                        d_list.append(tmp)
                        m_list.append('OFF')
                        la_list.append(0)
                        be_list.append(0)
                        if subscan[i] == '1' or '2':
                            s_list.append(1)
                        else:
                            s_list.append(2)
                        tmp = numpy.zeros(32768)
                    else:
                        pass
            elif mode[i] == 'HOT':
                #tmp += pickle.loads(d_[i])
                tmp += d_[i]
                if i == len(d_)-1:
                    d_list.append(tmp)
                    m_list.append('HOT')
                    la_list.append(0)
                    be_list.append(0)
                    if subscan[i] == 1 or 2:
                        s_list.append(1)
                    else:
                        s_list.append(2)
                    tmp = numpy.zeros(32768)
                else:
                    if mode[i+1] != 'HOT':
                        d_list.append(tmp)
                        m_list.append('HOT')
                        la_list.append(0)
                        be_list.append(0)
                        if subscan[i] == 1 or 2:
                            s_list.append(1)
                        else:
                            s_list.append(2)
                        tmp = numpy.zeros(32768)
                    else:
                        pass
            
        data_list.append(d_list)
        mode_list.append(m_list)
        lam.append(la_list)
        bet.append(be_list)
        scan_list.append(s_list)

    ret1 = calc_integdata(1, data_list, mode_list, lam, bet, scan_list, integ_mi, integ_ma)
    
    xscan_Ta = ret1[0]
    xscan_x = ret1[1]
    xscan_y = ret1[2]
    yscan_Ta = ret1[3]
    yscan_x = ret1[4]
    yscan_y = ret1[5]

    
    
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

        center_az_x = (xscan_x[20]+xscan_x[41])/2#xscan
        center_el_x = (xscan_y[20]+xscan_y[41])/2#xscan
        center_az_y = (yscan_x[20]+yscan_x[41])/2#yscan
        center_el_y = (yscan_y[20]+yscan_y[41])/2#yscan

        f_ = open(directory+'/hosei_copy', 'r')
        hosei_parameter = f_.read()
        f_.close()
        hosei_parameter = hosei_parameter.split('\n')

        return {'dAz':dAz, 'dEl':dEl, 'center_az_x':center_az_x, 'center_el_x':center_el_x,'center_az_y':center_az_y, 'center_el_y':center_el_y, 'hosei':hosei_parameter}


if __name__ == '__main__':
    ###
    f_1 = open('test_pointing.txt', 'a')
    p_dict = {}
    ###template fot result pointing.txt
    template = '{data_dir}  {daz_radio}  {del_radio}  {az_scan}  {az_azscan}  {el_azscan}  '\
               '{az_offset}  {el_scan}  {az_elscan}  {el_elscan}  {el_offset}  {az_integ_offset}  '\
               '{el_integ_offset} '
    for i in dir_list:
        data_path = i+'/'+i.split('/')[1]+'.fits'
        try:
            ret = analysis(i, data_path, 5000, 15000, 500, 8000, 9000)
        except Exception as e:
            print(data_path)
            print(e)
            continue
        p_dict['data_dir'] = i.split('/')[1]
        p_dict['daz_radio'] = float(ret['hosei'][16])
        p_dict['del_radio'] = float(ret['hosei'][17])
        p_dict['az_scan'] = 1 #???
        p_dict['az_azscan'] = ret['center_az_x']
        p_dict['el_azscan'] = ret['center_el_x']
        p_dict['az_offset'] = ret['dAz']
        p_dict['el_scan'] = 1#????
        p_dict['az_elscan'] = ret['center_az_y']
        p_dict['el_elscan'] = ret['center_el_y']
        p_dict['el_offset'] = ret['dEl']
        p_dict['az_integ_offset'] = float(ret['hosei'][16]) - ret['dAz'] - (-232.753)
        p_dict['el_integ_offset'] = float(ret['hosei'][17]) - ret['dEl'] - 315.131
        #p_dict['thot'] = ret['thot'][0]#check!!!
        write_str = template.format(**p_dict)
        
        print(write_str+'\n')
        f_1.write(write_str+'\n')

    f_1.close()
