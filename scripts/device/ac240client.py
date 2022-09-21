
import os
import socket
import struct

msg_fmt = 'd16384fdddddd'
msg_size = struct.calcsize(msg_fmt)

def connect(host, port, callback, args=[], kwargs={}, unpack=True):
    s = socket.socket()
    s.connect((host, port))
    
    while True:
        try:
            received = 0
            d = b''
            
            while received != msg_size:
                d += s.recv(msg_size - received)
                received = len(d)
                continue
            
            if unpack:
                d = msg_unpack(d)
                pass
            
            callback(d, *args, **kwargs)
        
        except KeyboardInterrupt:
            break;

        continue

    s.close()
    
    return


def msg_unpack(buff):
    ud = struct.unpack(msg_fmt, buff)
    dd = {
        'timestamp': ud[0],
        'spectrum': ud[1:16385],
        'total_power': ud[16385+0],
        'integ_time': ud[16385+1],
        'temp_board': int(ud[16385+2]),
        'temp_fpga': ud[16385+3],
        'overflow_fpga': int(ud[16385+4]),
        'overflow_ad': ud[16385+5],
    }
    return dd
    

def print_status(d, *args, **kwargs):
    print(f"{d['timestamp']} dt={d['integ_time']} tp={d['total_power']} overflow: ad={d['overflow_ad']} fpga={d['overflow_fpga']} temp: board={d['temp_board']} fpga={d['temp_fpga']}")
    return

def save_received_data(d, *args, **kwargs):
    f = open(kwargs['saveto'], 'ab')
    f.write(d)
    return

def open_saved_data(path):
    f = open(path, 'rb')
    f.seek(0, os.SEEK_END)
    size = f.tell()
    f.seek(0, os.SEEK_SET)

    d = []
    for i in range(int(size/msg_size)):
        d.append(
	    msg_unpack(
                f.read(msg_size)
            )
        )
        continue

    f.close()
    
    return d
    

