import struct
import time
import numpy
import os
import gc
import mmap
from tqdm import tqdm

class File():
    ###config
    header_size = 0#will be changed
    #d_format = "d 32768f 4s i i i"
    d_format = "d 655360f 4s i i i"
    
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

    np_dtype = numpy.dtype([('time', '<d'),
                            ('array1', ('<f', 32768)),#32768 1If
                            ('array2', ('<f', 32768)),
                            ('array3', ('<f', 32768)),
                            ('array4', ('<f', 32768)),
                            ('array5', ('<f', 32768)),
                            ('array6', ('<f', 32768)),
                            ('array7', ('<f', 32768)),
                            ('array8', ('<f', 32768)),
                            ('array9', ('<f', 32768)),
                            ('array10', ('<f', 32768)),
                            ('array11', ('<f', 32768)),
                            ('array12', ('<f', 32768)),
                            ('array13', ('<f', 32768)),
                            ('array14', ('<f', 32768)),
                            ('array15', ('<f', 32768)),
                            ('array16', ('<f', 32768)),
                            ('array17', ('<f', 32768)),
                            ('array18', ('<f', 32768)),                                                        
                            ('array19', ('<f', 32768)),
                            ('array20', ('<f', 32768)),
                            ('obs_mode', 'S4'),
                            ('a', '<i'),
                            ('b', '<i'),
                            ('c', '<i')])
    
    def __init__(self, path):
        self.path = path
        self.d_format = "d 32768f 4s i i i"
        self.st = struct.Struct(self.d_format)
        ###file
        self.f = open(path, "r+b")
        self.chunk = 8 + 655360*4 +4*4
        self.f_size = os.path.getsize(self.path)
        ###mmap test
        self.mm = mmap.mmap(self.f.fileno(), 0)
        
    def __len__(self):
        return int(self.f_size/self.chunk)

    def __getitem__(self, x):
        d_keys = ["timestamp",
                  "array1",
                  "array2",
                  "array3",
                  "array4",
                  "array5",
                  "array6",
                  "array7",
                  "array8",
                  "array9",
                  "array10",
                  "array11",
                  "array12",
                  "array13",
                  "array14",
                  "array15",
                  "array16",
                  "array17",
                  "array18",
                  "array19",
                  "array20",
                  "obs_mode",
                  "scan_num",
                  "lamdel",
                  "betdel"]
        if x > int(self.f_size/self.chunk):#need check...
            raise IndexError
        #self.f.seek(self.chunk * x)#mmap test
        #tmp = self.f.read(self.chunk)##mmap test
        self.mm.seek(self.chunk*x)
        tmp = self.mm.read(self.chunk)
        #return self._arange_list(self.st.unpack(tmp))
        data = numpy.frombuffer(tmp, self.np_dtype)
        data = list(data[0])
        data[21] = data[21].decode().replace("\x00", "")
        return dict(zip(d_keys,data))
        
    def read_timestamp(self):
        return self._read_partly(8, 0, "d")

    def read_obs_mode(self):
        return list(map(lambda x : x.decode().replace("\x00", ""), self._read_partly(4, 8 + 655360*4, "4s")))

    def read_scan_num(self):
        return self._read_partly(4, 8 + 655360*4+4, "i")

    def read_lamdel(self):
        return self._read_partly(4, 8 + 655360*4+4+4, "i")

    def read_betdel(self):
        return self._read_partly(4, 8 + 655360*4+4+4+4, "i")
        

    def _read_partly(self, length, offset, dtype):
        self.f.seek(0)
        tmp = []
        f_size = os.path.getsize(self.path)
        for i in tqdm(range(int(self.f_size/self.chunk))):
            self.f.seek(self.chunk*i + offset)
            tmp.append(self.f.read(length))
        return [struct.unpack(dtype, i)[0] for i in tmp]
        #return numpy.frombuffer(tmp, self.np_dtype)[0]

    def _arange_list(self,d):
        tp = list(d[1:32769])
        return [d[0], tp, d[32769].decode().replace("\x00", ""), d[32770], d[32771], d[32772]]
                                
    def read_all(self):
        self.f.seek(0)#need?
        d = self.f.read()
        tmp = [self.st.unpack(d[chunk*i:chunk*(i+1)]) for i in tqdm(range(int(self.f_size/self.chunk)))]
        return list(map(self._arange_list, tmp)) 

    def read_all2(self):
        self.mm.seek(0)
        d = self.mm.read()
        tmp = [self.st.unpack(d[chunk*i:chunk*(i+1)]) for i in tqdm(range(int(self.f_size/self.chunk)))]
        return list(map(self._arange_list, tmp))
                            
if __name__ == "__main__":
    path = "/home/amigos/ros/src/necst/scripts/record/hdd/test5.dat"
    ww = Read(path)
    d = ww.read_all()

            
