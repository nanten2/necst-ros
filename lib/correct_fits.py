#!/usr/bin/env python3
import astropy.io.fits
import time

def calc(fits):
	print("*** " +fits +" ***")

	f = astropy.io.fits.open(fits)[1]
		
	if "12CO" in fits:
		SYNTH = [8018.4179859 for i in range(len(f.data["DATA"]))]
		CRVAL1 = [i * 16382/1000 * (8018.4179859-8038.000000000) for i in f.data["CDELT1"]]
	else:
		SYNTH = [9320.2397758 for i in range(len(f.data["DATA"]))]
		CRVAL1 = [i * 16382/1000 * (9320.2397758-9301.318999999) for i in f.data["CDELT1"]]	
	f.data["SYNTH"] = SYNTH
	f.data['_2NDLO'] = SYNTH
	f.data["CRVAL1"] = CRVAL1 + f.data["VFRAME"]
		
	ftime = str(time.time())
	savefile = fits.rsplit(".",1)[0] + "_test"+ftime+".fits"
	cols = astropy.io.fits.ColDefs(f.data)
	tbhdu = astropy.io.fits.BinTableHDU.from_columns(cols)
	tbhdu.writeto(savefile)
	print("save : ", savefile)
	return savefile

if __name__=="__main__":
	print("start program")
	fits = input("fits : ")
	calc(fits)
	print("end program")



