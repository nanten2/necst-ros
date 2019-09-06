#!/usr/bin/env python3
import glob
###
key_word = '20181114'
###

dir_list = glob.glob('/home/amigos/data/radio_pointing_9/*{}*pointing'.format(key_word))
print(dir_list)

#----
import sys
sys.path.append('/home/amigos/ros/src/necst/lib')
import n2df
import numpy
from scipy.optimize import curve_fit

#-----
def f(x, a, b, c):
    return a*x**2 + b*x + c

def gaussian(x, a, mu, gamma):
    return a * numpy.exp(- gamma * (x - mu) **2)

para_init = numpy.array([25000., 0.1, 0.0001])

def calc_integdata(IF, data_list, mode_list, lam, bet, scan_list, mi, ma, width, integ_mi, integ_ma):

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

    tmp = []
    for i in range(len(hotmask)):
        if hotmask[i] == 1:
            if len(tmp) == 0:
                for j in range(i+1):
                    tmp.append(data_list[i])
            else:
                tmp.append(data_list[i])
        else:
            if len(tmp) == 0:
                pass
            else:
                tmp.append(tmp[-1])
    HOTlist = numpy.array(tmp)
    
    tmp = []
    for i in range(len(offmask)):
        if offmask[i] == 1:
            if len(tmp) == 0:
                for j in range(i+1):
                    tmp.append(data_list[i])
            else:
                tmp.append(data_list[i])
        else:
            if len(tmp) == 0:
                pass
            else:
                tmp.append(tmp[-1])
    OFFlist = numpy.array(tmp)
    
    ONlist = numpy.array(data_list)
    
    Taslist = (ONlist - OFFlist)/(HOTlist - OFFlist) * 300
    
    x = numpy.linspace(0, 32768, 32768)

    rTaslist_tmp = []
    rtmp = []
    for i in range(len(Taslist)):
        base = []
        start = int(numpy.argmax(Taslist[i][int(mi):int(ma)]) + (mi - width))
        end = int(numpy.argmax(Taslist[i][int(mi):int(ma)]) + (mi + width))
        dif = end - start
        base.extend(Taslist[i])
        base[start:end] = []
        param = numpy.polyfit(x[:32768-dif], base, 2)
        rTas = Taslist[i] - f(x, *param)
        rTaslist_tmp.append(rTas)
    rTaslist = numpy.array(rTaslist_tmp)
    
# create data for plot
    xscan_Ta = []
    xscan_x = []
    xscan_y = []
    
    yscan_Ta = []
    yscan_x = []
    yscan_y = []

    for i in range(len(xmask)):
        if xmask[i] == 1:
            xscan_Ta.append(rTaslist[i])
            xscan_x.append(lam[i])
            xscan_y.append(bet[i])
        else:
            pass

    for i in range(len(ymask)):
        if ymask[i] == 1:
            yscan_Ta.append(rTaslist[i])
            yscan_x.append(lam[i])
            yscan_y.append(bet[i])
        else:
            pass

    # TA* integration
    xscan_integ = []
    yscan_integ = []
    for i in range(len(xscan_Ta)):
        lx = xscan_Ta[i]
        xscan_integ.append(numpy.sum(lx[int(integ_mi):int(integ_ma)]))

    for i in range(len(yscan_Ta)):
        ly = yscan_Ta[i]
        yscan_integ.append(numpy.sum(ly[int(integ_mi):int(integ_ma)]))

    return xscan_integ, xscan_x, xscan_y, yscan_integ, yscan_x, yscan_y, xscan_Ta, yscan_Ta


para_init = numpy.array([25000., 0.1, 0.0001])

    #-----
def analysis(IF, directory, file_name, mi=10000, ma=30000, width=1000, integ_mi=15500, integ_ma=17500):
# open file

    n = n2df.Read(file_name)    
    _n = n.read_all()
    d = []
    for i in range(25):
        _d = []
        for j in range(len(_n)):
            _d.append(_n[j][i])
        d.append(_d)
        
# define axis 
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
                tmp += d_[i]
                if subscan[i+1] == 2 or mode[i+1] == 'OFF' or mode[i+1] == 'HOT':
                    d_list.append(tmp)
                    m_list.append('ON')
                    la_list.append(_lam[i])
                    be_list.append(_bet[i])
                    s_list.append(1)
                    tmp = numpy.zeros(32768)
                else:
                    pass
            elif subscan[i] == 2 and mode[i] == 'ON':
                tmp += d_[i]
                if subscan[i+1] == 1 or mode[i+1] == 'OFF' or mode[i+1] == 'HOT':
                    d_list.append(tmp)
                    m_list.append('ON')
                    la_list.append(_lam[i])
                    be_list.append(_bet[i])
                    s_list.append(2)
                    tmp = numpy.zeros(32768)
                else:
                    pass
            elif mode[i] == 'OFF':
                tmp += d_[i]
                if mode[i+1] == 'ON' or mode[i+1] == 'HOT':
                    d_list.append(tmp)
                    m_list.append('OFF')
                    la_list.append(0)
                    be_list.append(0)
                    if subscan[i] == 1:
                        s_list.append(1)
                    else:
                        s_list.append(2)
                    tmp = numpy.zeros(32768)
                else:
                    pass       
            elif mode[i] == 'HOT':
                tmp += d_[i]
                if i == len(d_)-1:
                    d_list.append(tmp)
                    m_list.append('HOT')
                    la_list.append(0)
                    be_list.append(0)
                    if subscan[i] == 1:
                        s_list.append(1)
                    else:
                        s_list.append(2)
                    tmp = numpy.zeros(32768)
                else:
                    if mode[i+1] == 'ON' or mode[i+1] == 'OFF':
                        d_list.append(tmp)
                        m_list.append('HOT')
                        la_list.append(0)
                        be_list.append(0)
                        if subscan[i] == 1:
                            s_list.append(1)
                        else:
                            s_list.append(2)
                        tmp = numpy.zeros(32768)
                    else:
                        pass
            else:
                print("check")
        data_list.append(d_list)
        mode_list.append(m_list)
        lam.append(la_list)
        bet.append(be_list)
        scan_list.append(s_list)
    
    ret1 = calc_integdata(IF, data_list, mode_list, lam, bet, scan_list, mi, ma, width, integ_mi, integ_ma)

    xscan_integ = ret1[0]
    xscan_x = ret1[1]
    xscan_y = ret1[2]
    yscan_integ = ret1[3]
    yscan_x = ret1[4]
    yscan_y = ret1[5]
    xscan_Ta = ret1[6]
    yscan_Ta = ret1[7]


# Gaussian Fitting function
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

        center_az_x = xscan_x[:5][2]#xscan
        center_el_x = xscan_y[:5][2]#xscan
        center_az_y = yscan_x[5:][2]#yscan
        center_el_y = yscan_y[5:][2]#yscan
        
        f_ = open(directory+'/hosei_copy', 'r')
        hosei_parameter = f_.read()
        f_.close()
        hosei_parameter = hosei_parameter.split('\n')
        #print(hosei_parameter)
        

        return {'dAz':dAz, 'dEl':dEl, 'center_az_x':center_az_x, 'center_el_x':center_el_x,'center_az_y':center_az_y, 'center_el_y':center_el_y, 'hosei':hosei_parameter}

if __name__ == '__main__':
    ###
    f_1 = open('test_pointing.txt', 'a')
    p_dict = {}
    IF = 0
    ###template fot result pointing.txt
    template = '{data_dir}  {daz_radio}  {del_radio}  {az_scan}  {az_azscan}  {el_azscan}  '\
               '{az_offset}  {el_scan}  {az_elscan}  {el_elscan}  {el_offset}  {az_integ_offset}  '\
               '{el_integ_offset} '
    for i in dir_list:
        data_path = i+'/'+i.split('/')[1]+'.fits'
        try:
            ret = analysis(IF, i, data_path, 5000, 15000, 500, 8000, 9000)
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
