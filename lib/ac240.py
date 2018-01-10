
""" -- comment --
:: 1p85_server python client ::

dfs.py

[history]
2010/11/02 2010ver created nishimura

"""

# ------ SETTINGS ------

WINDOW_NONE      = 0
WINDOW_HANNING   = 1
WINDOW_HAMMING   = 2
WINDOW_FLATTOP   = 3
WINDOW_BLACKMANN = 4

REPLY_SPECTRUM    = 'BINARY16KDOUBLE_SIGNAL'
REPLY_AD          = 'BINARY500KCHAR_SIGNAL'
REPLY_STATUS      = 'STATUS_SIGNAL'
REPLY_TIMESTAMP   = 'TIMESTAMP_SIGNAL'
REPLY_TEMPERATURE = 'TEMPERATURE_SIGNAL' 

HELP_STR = """
[ --- AC240 USAGE --- ]
 getspectrum(repeat,integsec,starttime)  : get spectrum data
 getdata()                               : return spectrum data
 getad(--)                               : get A/D data
 getstatus()                             : get machine status
 gettemperature()                        : get temperature of a board and chip
 initialize()                            : do initialize  
 caliblation()                           : do A/D caliblation
 finalize()                              : do finalize
 setfpgawindowfunc(func)                 : set FPGA window function
 setfpgaintegmsec(integmsec)             : set FPGA integration time
 setadfullscale(volt)                    : set A/D fullscale range
 setoffset(offset)                       : set A/D offset
 setadcoupling(coupling)                 : set A/D coupling
 setadbandwidth(bandwidth)               : set A/D bandwidth
"""



# ------ start source code ------

import struct, numpy
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
import dfs_table
import dfs_client

DB_LOG     = 'log_'+ dfs_table.DFS_TABLE +'_server'
DB_MONITOR = 'monitor_' + dfs_table.DFS_TABLE


class ac240(dfs_client.Client):
    def __init__(self, host, port, print_socket=True):
        dfs_client.Client.__init__(self, print_socket=print_socket)
        self._set_reply_handler(self.__ac240_reply_handler)
        self._init_socket(host, port)
        self._init_db(table_log=DB_LOG, table_monitor=DB_MONITOR)
        
        # dfs setting
        self._data = []
        self._status = range(6)
        self._temperature = range(3)
        self._windowfunc = ('none', 'hanning', 'hamming', 'flattop', 'blackmann')
        self._coupling = ('GND', 'DC 1Mohm', 'AC 1Mohm', 'DC 50ohm', 'AC 50ohm')
        self._bandwidth = (1000, 25, 700, 200, 20, 35)
        return
    
    def help(self):
        self._print_common_help()
        print(HELP_STR)
        return

    def __ac240_reply_handler(self, reply):
        if reply==REPLY_SPECTRUM: self.__reply_spectrum()
        elif reply==REPLY_AD: self.__reply_ad()
        elif reply==REPLY_STATUS: self.__reply_status()
        elif reply==REPLY_TIMESTAMP: self.__reply_timestamp()
        elif reply==REPLY_TEMPERATURE: self.__reply_temperature()
        else: self.__reply_else(reply)
        return
    
    def __reply_else(self, reply):
        if self.print_socket==True: print(reply)
        return
    
    def __reply_spectrum(self):
        num = 16384
        self._data.append(struct.unpack('='+'d'*num, self._socket.read_reply_bin(num*8)))
        return
    
    def __reply_ad(self):
        pass
    
    def __reply_status(self):
        s = self._socket.read_reply_line().split()
        self._status[0] = float(s[0])    # FPGA integ msec 
        self._status[1] = int(s[1])      # FPGA WindowFunction 
        self._status[2] = float(s[2])    # A/D Fullscale V
        self._status[3] = float(s[3])    # A/D Offset V 
        self._status[4] = int(s[4])      # A/D Coupling
        self._status[5] = int(s[5])      # A/D BandWidth 
        return
    
    def __reply_timestamp(self):
        pass
    
    def __reply_temperature(self):
        s = self._socket.read_reply_line().split()
        self._temperature[0] = float(s[0]) # timestamp
        self._temperature[1] = float(s[1]) # FPGA temperature
        self._temperature[2] = float(s[2]) # Board temperature
        return
    
    def __print_status(self):
        s = self._status
        print('[FPGA] Integ:%lfmsec, Window:%s,\n'%(s[0]*1000, self._windowfunc[s[1]])+
              '[A/D] Fullscale:%lfV, Offset:%lfV, Coupling:%s, Bandwidth:%dMHz'%(s[2], s[3], self._coupling[s[4]], self._bandwidth[s[5]]))
        return

    def __print_temperature(self):
        s = self._temperature
        print('[Temperature] FPGA:%lf, Board:%lf, timestamp:%lf'%(s[1], s[2], s[0]))
        return
    
    def getspectrum(self, repeat=1, integsec=1.0, starttime=0.0):
        """Get spectrum data. repeat=1, integsec=1.0, starttime=0.0"""
        self._send_command('AC240:spectrum_getn', '%ld %lf %lf'%(repeat, starttime, integsec))
        self._read_reply_loop()
        return

    def getstatus(self):
        self._send_command('AC240:status_get')
        self._read_reply_loop()
        self.__print_status()
        return self._status

    def getdata(self):
        ret_data = numpy.array(self._data)
        self._data = []
        return ret_data

    def gettemperature(self):
        self._send_command('AC240:temperature_get')
        self._read_reply_loop()
        self.__print_temperature()
        return

    def caliblation(self):
        self._send_command('AC240:caliblation')
        self._read_reply_loop()
        return
    
    def initialize(self):
        self._send_command('AC240:init')
        self._read_reply_loop()
        return
    
    def finalize(self):
        self._send_command('AC240:finalize')
        self._read_reply_loop()
        return

    """
    def serverstop(self):
        self._send_command('AC240:server_stop')
        self._read_reply_loop()
        return
    """

    def setfpgawindowfunc(self, func=4):
        if func<0 or func>4: print('bad func num'); print(self._windowfunc);return
        self._send_command('AC240:status_set_fpga', '2 0.0 %ld'%(int(func)))
        self._read_reply_loop()
        self.getstatus()
        return

    def setfpgaintegmsec(self, integmsec=100):
        self._send_command('AC240:status_set_fpga', '1 %lf 0'%(float(integmsec)/1000.))
        self._read_reply_loop()
        self.getstatus()
        return

    def setadfullscale(self, volt=5):
        self._send_command('AC240:status_set_ad', '1 %lf 0.0 0 0'%(float(volt)))
        self._read_reply_loop()
        self.getstatus()
        return
    
    def setadoffset(self, offset=0.0):
        self._send_command('AC240:status_set_ad', '2 0.0 %lf 0 0'%(float(offset)))
        self._read_reply_loop()
        self.getstatus()
        return
    
    def setadcoupling(self, coupling=2):
        self._send_command('AC240:status_set_ad', '4 0.0 0.0 %ld 0'%(int(coupling)))
        self._read_reply_loop()
        self.getstatus()
        return

    def setadbandwidth(self, bandwidth=0):
        self._send_command('AC240:status_set_ad', '8 0.0 0.0 0 %ld'%(int(bandwidth)))
        self._read_reply_loop()
        self.getstatus()
        return


    
def cancel():
    a = ac240()
    a.cancel()
    return
  
