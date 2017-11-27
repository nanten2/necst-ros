import time
import sys
import serial
from serial.tools import list_ports

num=1000

#ser = serial.Serial("/dev/ttyUSB0", baudrate = 38400, timeout = 1)
#ser.write(b"B")
#ser.close()



def enc(device):
    ser = serial.Serial(device, baudrate = 38400, timeout = 1)
    ch1 = ''
    ch2 = ''
    ft = time.time()
    ser.write(b"P")
    st = time.time()
    ser.read(1).decode('utf-8')
    tt = time.time()
    """
    for i in range(27):
        if 1 < i < 13:
            ch1 += ser.read(1).decode('utf-8')
        elif 15 < i < 27:
            ch2 += ser.read(1).decode('utf-8')
        else:
            trash = ser.read(1).decode('utf-8')
    #bt = time.time()
    #print(bt-at)
    #ser.write(b"F")
    """
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
for i in range(int(num)):
    count = enc(device)
    sum1 += count[0]
    sum2 += count[1]
    sum3 += count[2]
    #print("count",count[0], count[1])
    """
    pls = enc()
    az = int(pls[0])*0.12
    el = int(pls[1])*0.12
    deg_az = az/3600.
    deg_el = el/3600.+ 45. 
    print(deg_az, deg_el)
    time.sleep(0.1)
    #ser.write(b"F")
    """
    
b = time.time()
#print((b-a)/100.)
print("write", sum1/num)
print("read",sum2/num)
print("write&read",sum3/num)
