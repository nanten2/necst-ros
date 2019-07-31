import struct
import time
import numpy
import os
import gc
from tqdm import tqdm

class File():
    ###config
    header_size = 0#will be changed
    d_format = "d 32768f 4s i i i"
    
    def __init__(self, path):
        self.f = open(path, "ab")
        self.st = struct.Struct(self.d_format)
        pass

    def write(self, data):
        tmp = [data[0], *data[1] ,data[2].encode(), data[3], data[4], data[5]]
        self.f.write(self.st.pack(*tmp))
        print("check")

    def open(self):
        self.f = open(path, "wb")
        
    def close(self):
        self.f.close()
    

class Read():
    data_type = {"timestamp" : "d",
                 "array" : "32768f",
                 "obs_mode" : "4s",
                 "scan_num" : "i",
                 "lamdel" : "i",
                 "betdel" : "i"}
    
    def __init__(self, path):
        self.path = path
        self.d_format = "d 32768f 4s i i i"
        self.st = struct.Struct(self.d_format)
        ###file
        self.f = open(path, "rb")
        self.chunk = 8 + 32768*4 +4*4
        self.f_size = os.path.getsize(self.path)
        
    def __len__(self):
        return int(self.f_size/self.chunk)

    def __getitem__(self, x):
        if x > int(self.f_size/self.chunk):#need check...
            raise IndexError
        self.f.seek(self.chunk * x)
        tmp = self.f.read(self.chunk)
        return self._arange_list(self.st.unpack(tmp))

    def read_timestamp(self):
        return self._read_partly(8, 0, "d")

    def read_obs_mode(self):
        return self._read_partly(4, 8 + 32768*4, "4s")

    def _read_partly(self, length, offset, dtype):
        self.f.seek(0)
        chunk_size = 8 + 32768*4 + 4*4
        tmp = []
        f_size = os.path.getsize(self.path)
        for i in range(int(f_size/chunk_size)):
            self.f.seek(chunk_size*i + offset)
            tmp.append(self.f.read(length))
        return [struct.unpack(dtype, i)[0] for i in tmp]

    def _arange_list(self,d):
        tp = list(d[1:32769])
        return [d[0], tp, d[32769].decode().replace("\x00", ""), d[32770], d[32771], d[32772]]
                                
    
    def read_all(self):
        self.f.seek(0)#need?
        d = self.f.read()
        f_size = os.path.getsize(self.path)
        chunk = 8 + 32768*4 +4*4
        tmp = [self.st.unpack(d[chunk*i:chunk*(i+1)]) for i in tqdm(range(int(f_size/chunk)))]
        return list(map(_arange_list, tmp)) 


if __name__ == "__main__":
    path = "/home/amigos/ros/src/necst/scripts/record/hdd/test5.dat"
    ww = Read(path)
    d = ww.read_all()
    for i in range(4):
        print(d[i][2])
            
