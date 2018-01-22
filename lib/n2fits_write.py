# coding:utf-8
from astropy.io import fits
import numpy 
      

def write(d,f):
    '''
    n2fits_write.write(d,f)
       	d:observasion data(72data)
       	f:save file('name.fits')
    '''

    #d['TUNIT6'] = ['km/s']#固定値
    #d['CTYPE1'] = ['km/s']
    #d['CTYPE2'] = ['deg']
    #d['CTYPE3'] = ['deg']
    #d['T_VLSR'] = ['0']
    #d['SCAN'] = ['1']
    #d['BEAMFF'] = ['1']
    #d['SIG'] = ['T']
    #d['CAL'] = ['F']
    #d['QUALITY'] = ['1']
    #d['OTFVLAM'] = ['0']
    #d['SUBSCAN'] = ['1']
    #d['LOCKSTAT'] = ['F']

    keyfmtuni = [
        ('OBJECT', '16A',''),
        ('BANDWID','1D','Hz'),
        ('DATE-OBS','22A',''),
        ('EXPOSURE','1D','s'),
        ('TSYS','1D','K'),
        ('DATA','16384E','K'),#新16384E#旧4097E
        ('TDIM6','3E',''),
        ('TUNIT6','16A',''),
        ('CTYPE1','8A',''),
        ('CRVAL1','1D',''),
        ('CRPIX1','1D',''),
        ('CDELT1','1D',''),
        ('CTYPE2','8A',''),
        ('CRVAL2','1D',''),
        ('CTYPE3','8A',''),
        ('CRVAL3','1D',''),
        ('T_VLSR','1D',''),
        ('OBSERVER','16A',''),
        ('SCAN','1J',''),
        ('OBSMODE','8A',''),
        ('MOLECULE','16A',''),
        ('TRANSITI','16A',''),
        ('TEMPSCAL','16A',''),
        ('FRONTEND','16A',''),
        ('BACKEND','16A',''),
        ('THOT','1D','K',),
        ('TCOLD','1D','K',),
        ('FREQRES','1D','MHz'),
        ('TIMESYS','16A',''),
        ('VELDEF','8A',''),
        ('VFRAME','1D','km/s'),
        ('VFRAME2','1D','km/s'),
        ('OBSFREQ','1D','MHz'),
        ('IMAGFREQ','1D','MHz'),
        ('LST','1J','sec'),
        ('AZIMUTH','1D','deg'),
        ('ELEVATIO','1D','deg'),
        ('TAU','1D',''),
        ('HUMIDITY','1D','%'),
        ('TAMBIENT','1D','K'),
        ('PRESSURE','1D','mm Hg'),
        ('WINDSPEE','1D','m/s'),
        ('WINDDIRE','1D','deg'),
        ('BEAMEFF','1D',''),
        ('RESTFREQ','1D','MHz'),
        ('SIG','1A',''),
        ('CAL','1A',''),
        ('SOBSMODE','8A',''),
        ('QUALITY','1J',''),
        ('AOSLEN','1D','s'),
        ('LOFREQ','1D','MHz'),
        ('SYNTH','1D','MHz'),
        ('FREQSWAM','1D','Hz'),
        ('COORDSYS','8A',''),
        ('COSYDEL','8A',''),
        ('LAMDEL','1D','arcsec'),
        ('BETDEL','1D','arcsec'),
        ('OTADEL','1A',''),
        ('OTFVLAM','1D','arcsec/s'),
        ('OTFVBET','1D','arcsec/s'),
        ('OTFSCANN','1D',''),
        ('OTFLEN','1D','s'),
        ('SUBSCAN','1J',''),
        ('MJD','1J','day'),
        ('SECOFDAY','1D','s'),
        ('SIDEBAND','1A',''),
        ('_2NDSB','1A',''),
        ('_3RDSB','1A',''),
        ('_2NDLO','1D','MHz'),
        ('_3RDLO','1D','MHz'),
        ('SUBREF','1D',''),
        ('LOCKSTAT','1A',''),
        ]

    #print(len(d['DATA'][0]))
    col = []#col0~col72
    le = int(len(d['DATA']))
    for i in range(0,72):
        if i == 6:
            print(str(type(d[key])))
        key, fmt, uni = keyfmtuni[i]
        if type(d[key]) == numpy.ndarray:
            new = d[key]
        elif type(d[key]) == list:
            new = d[key]
        else:
            new = [d[key]]*le
        col.append(fits.Column(name=key, format=fmt, unit=uni, array=new))
    
    cols = fits.ColDefs(col)
    tbhdu = fits.BinTableHDU.from_columns(cols)
    print(cols)
    tbhdu.writeto(f)

    
    return
