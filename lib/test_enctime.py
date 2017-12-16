import time
import sys
import serial
from serial.tools import list_ports
from datetime import datetime as dt

num=1000

def enc(device):
    ser = serial.Serial(device, baudrate = 38400, timeout = 1)
    ch1 = ''
    ch2 = ''
    ft = time.time()
    ser.write(b"P")
    st = time.time()
    ser.read(1).decode('utf-8')
    tt = time.time()

    return [st-ft,tt-st,tt-ft]


a = time.time()
sum1 = 0
sum2 = 0
sum3 = 0
port = list_ports.comports()
for info in port:
    device = info.device
if device:
    pass
else:
    print("error!! : No device")
    sys.exit()
for j in range(100):
    aa = dt.utcnow()
    name = "./enc_log/" + aa.strftime("%H%m%S%f") + ".txt"
    
    for i in range(int(num)):
        count = enc(device)
        with open(name,"a") as f:
            f.write(str(count[0])+" "+str(count[1])+" "+str(count[2])+"\n")
        sum1 += count[0]
        sum2 += count[1]
        sum3 += count[2]

print("write", sum1/num)
print("read",sum2/num)
print("write&read",sum3/num)
