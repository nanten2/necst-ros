import time
import numpy as np
import sys
sys.path.append("/home/amigos/git/")
from n2db import n2database
db = n2database.N2db()
db.authorize()

def read():
     st=time.time()
     data=db.SELECT("AzEl", "2018-05-18","2018-05-18",num=1)
     dt = time.time()-st
     aa.append(dt)
     ave = np.average(aa)
     return ("time", dt, "average", ave, "num", len(aa))


if __name__ == "__main__":
    aa = []
    flag = True
    while flag:
        try:
            ret = read()
            print(ret)
            time.sleep(1.)
        except KeyboardInterrupt:
            print("tr+C!!")
            flag = False
            pass
        except Exception as e:
            print("error : ", e)
           
 


     
