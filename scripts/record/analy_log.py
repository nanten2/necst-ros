import sys
import matplotlib.pyplot as plt 

param = sys.argv
param.append("")

_time = []
com_az = []
com_el = []
enc_az = []
enc_el = []

try:
    f = open(param[1], "r")
except:
    f = open("/home/amigos/data/tk_log/"+param[1], "r")
ff = f.readlines()
del ff[0]#delete comment_out
for i in ff:
    data = i.split()
    _time.append(data[0])
    com_az.append(data[1])
    com_el.append(data[2])
    enc_az.append(data[3])
    enc_el.append(data[4])

print(len(_time))
print(len(enc_az))
plt.plot(_time, com_az,linestyle=None, marker=".", label="command_az")
plt.plot(_time, enc_az,linestyle=None, marker=".", label="encoder_az")
plt.grid()
plt.show()
