import pyinterface
from datetime import datetime as dt
import time

dio = pyinterface.open(2724, 0)


print("check start\n")
while True:
    now = dt.utcnow()
    date = now.strftime("UTC %Y/%m/%d %H:%M:%S")
    parameter = dio.input_dword().to_list()
    print(date, " ", parameter)    

    f = open("antenna_dio_data.txt", "a")
    f.write(date)
    f.write(" ")
    f.write(str(parameter))
    f.write("\n")
    f.close()
    time.sleep(1.)
