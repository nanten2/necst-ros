import time
import serial

#ser = serial.Serial("/dev/ttyUSB0", baudrate = 38400, timeout = 1)
#ser.write(b"B")
#ser.close()

def enc():
    ser = serial.Serial("/dev/ttyUSB0", baudrate = 38400, timeout = 1)
    ch1 = ''
    ch2 = ''
    #at = time.time()
    ser.write(b"P")
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
    return [ch1, ch2]

"""
a = time.time() 
for i in range(10000):
    pls = test()
    #pls1 = pls[2:13]
    #print(pls[0], pls[1])
    az = int(pls[0])*0.5
    el = int(pls[1])*0.5
    deg_az = az/3600.
    deg_el = el/3600.+ 45. 
    print(deg_az, deg_el)
    #ser.write(b"F")
b = time.time()
print((b-a)/100.)
"""
