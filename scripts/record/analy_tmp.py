import numpy
import n2lites as n2lite
import sqlite3
import matplotlib.pyplot as plt

###path
path_to_adb = "./hdd/write_test/otf_20190715_3_antenna.db"
path_to_db = "./hdd/write_test/otf20190715_3.db"

#read from DB
n1 = n2lite.xffts_logger(path_to_adb)
d1 = n1.read("encoder")
d1[0] = numpy.array(d1[0])
#tget imestamp from spec db
#n2 = n2lite.xffts_logger(path_to_db)
#print(n2.check_table())
n2 = sqlite3.connect(path_to_db)
a = n2.execute("select timestamp from xffts")
d2 = a.fetchall()


#azel -->radec
import azel_to_radec
#print(d1[0])
#print(d2)
az = []
el = []
index = []
for i in d2:
    try:
        #d2:xffts d1:antenna
        a = numpy.where(d1[0]>i[0])
        index.append(a[0][0])
        #break
    except Exception as e:
        print(i[0], e)

#print(index)

Az_list = numpy.array(d1[1])
diff_Az = Az_list[1:] - Az_list[:-1]
El_list = numpy.array(d1[2])
diff_El = El_list[1:] - El_list[:-1]
time_list = numpy.array(d1[0])
diff_time = time_list[1:] - time_list[:-1]

#print(Az_list)
#print(diff_Az)
#plt.plot(diff_El)
#plt.plot(diff_Az)
#plt.show()

n_az = []
n_el = []


for n,i in enumerate(index):
    try:
        n_az.append((diff_Az[i]/diff_time[i]) * (d2[n][0] - time_list[i-1]) + Az_list[i-1])
        n_el.append((diff_El[i]/diff_time[i]) * (d2[n][0] - time_list[i-1]) + El_list[i-1])
    except Exception as e:
        print(i)

        
#print(n_az)
#print(n_el)
n_t = []
for i in d2:
    n_t.append(i[0])

ret = azel_to_radec.fk5_from_altaz(numpy.array(n_az)/3600, numpy.array(n_el)/3600, n_t)
#tekitou
#print(ret[0].ra.deg)
#print(n_t)
ra = []
dec = []
for i in ret:
    ra.append(i.ra.deg)
    dec.append(i.dec.deg)

print(ret[0])

#check
plt.plot(ra, dec, ".")
plt.legend()
plt.grid()
plt.xlabel("Ra (J2000) [deg]")
plt.ylabel("Dec (J2000) [deg]")
plt.show()



t = []
for i in range(len(d2)):
    t.append(d2[i][0])
plt.plot(t, ".")
plt.show()

#check
ret = azel_to_radec.fk5_from_altaz(Az_list/3600, El_list/3600,time_list)
ra = []
dec = []
for i in ret:
    ra.append(i.ra.deg)
    dec.append(i.dec.deg)

print(ret[0])

#check
plt.plot(ra, dec, ".")
plt.legend()
plt.grid()
plt.xlabel("Ra (J2000) [deg]")
plt.ylabel("Dec (J2000) [deg]")
plt.show()
