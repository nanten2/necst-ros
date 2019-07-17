import sys
sys.path.append("/home/amigos/git")
import n2lites as n2lite
import pickle
import time
import numpy#for check, will be deleted

def _un_sirialize(a):
    b = pickle.loads(a)
    return b

def dis_sirialize(array):
    """
    return python list from pickled array
    
    parameter
    ==========
    array : pickled object
    
    reutrn
    ==========
    array : list
    """
    return list(map(_un_sirialize,array))


def read(path_to_db, table_name, timestamp=""):
    if timestamp == "":timestamp = time.time()
    n = n2lite.xffts_logger(path_to_db)
    data = n.read_as_timestamp(table_name, timestamp)
    timestamp = data[0]
    spec_data = dis_sirialize(data[1])#data[0] : timestamp data[1]:spec
    return timestamp ,spec_data

def read2(path_to_db, table_name, timestamp=""):
    if timestamp == "":timestamp = time.time()
    n = n2lite.xffts_logger(path_to_db)
    data = n.read_as_timestamp(table_name, timestamp, param = "timestamp")
    timestamp = data[0]
    #spec_data = dis_sirialize(data[1])#data[0] : timestamp data[1]:spec
    return timestamp


### test script (will be deleted)
if __name__ == "__main__":
    path_to_db = "./test11.db"
    table_name = "xffts"
    a = read(path_to_db, table_name, time.time())
    b = read2(path_to_db, table_name, time.time())
    print(b[-1])
