import math

def apply_kisa_test(az, el, hosei):
    """ from [coordinate.cpp]
    #kisa parameter
        (daz[0], de[1], kai_az[2], omega_az[3], eps[4], kai2_az[5], omega2_az[6], kai_el[7], omega_el[8], kai2_el[9], omega2_el[10], g[11], gg[12], ggg[13], gggg[14],
        del[15], de_radio[16], del_radio[17], cor_v[18], cor_p[19], g_radio[20], gg_radio[21], ggg_radio[22], gggg_radio[23])
    """
    kisa = read_kisa_file(hosei,24)
    DEG2RAD = math.pi/180.
    RAD2DEG = 180/math.pi
    ARCSEC2RAD = math.pi/(180*60*60.)
    kisa[3] = kisa[3]*DEG2RAD
    kisa[6] = kisa[6]*DEG2RAD
    kisa[8] = kisa[8]*DEG2RAD
    kisa[10] = kisa[10]*DEG2RAD
    kisa[19] = kisa[19]*DEG2RAD
    el_d = el*RAD2DEG
    delta = [0,0]

    # reference from src/coord/correct.
    # line 242, 248
    dx = kisa[2]*math.sin(kisa[3]-az)*math.sin(el)+kisa[4]*math.sin(el)+kisa[0]*math.cos(el)+kisa[1]+kisa[5]*math.cos(2*(kisa[3]-az))*math.sin(el)+kisa[16]+kisa[18]*math.cos(el+kisa[19])
    delta[0] = dx #
    dy = -kisa[7]*math.cos(kisa[8]-az)-kisa[9]*math.sin(2*(kisa[10]-az))+kisa[15]+kisa[11]*el_d+kisa[12]*el_d*el_d+kisa[13]*el_d*el_d*el_d+kisa[14]*el_d*el_d*el_d*el_d+kisa[17]-kisa[18]*math.sin(el+kisa[19])+kisa[20]*el_d+kisa[21]*el_d*el_d+kisa[22]*el_d*el_d*el_d+kisa[23]*el_d*el_d*el_d*el_d
    delta[1] = dy # arcsec
    if(math.fabs(math.cos(el))) > 0:
       delta[0]=delta[0]/math.cos(el)
        
    return delta

def read_kisa_file(hosei, num):
    f = open(hosei)
    line = f.readline()
    kisa = [0]*num
    n = 0
    while line:
        line = line.rstrip()
        kisa[n] = float(line)
        line = f.readline()
        n = n+1
    f.close()
    return kisa
