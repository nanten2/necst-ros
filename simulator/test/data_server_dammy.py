#!/usr/bin/env python

import sys
import time
from datetime import datetime
import rospy
import numpy as np
import signal
import threading
import random
from astropy.io import fits
from nascorx_xffts.msg import XFFTS_msg
from nascorx_xffts.msg import XFFTS_pm_msg
from nascorx_xffts.msg import XFFTS_temp_msg_du
from necst.msg import xffts_flag_msg


class data_server(object):
    header_size = 64
    BE_num_Max = 20
    _stop_loop = False

    def __init__(self):
        self.obs = sys.argv[1] # 'edge' or 'cross' or 'otf'
        rospy.init_node('XFFTS_data_server')
        rospy.Subscriber('XFFTS_DB_flag', xffts_flag_msg, self.flag, queue_size = 100)
        self.sample_rate = 30
        self.timestamp = 0
        self.newdb_name = ''
        self.obs_mode = ''
        self.scan_num = ''
        self.lamdel = 0
        self.betdel = 0
        self.data = 0
        self.on_count = 0
        self.off_count = 0
        self.hot_count = 0
        self.ind_list_on = 0
        self.ind_list_hot = 0
        self.ind_list_off = 0
        if self.obs == 'cross':
            self.hdu1 = fits.open('/home/amigos/backup/radio_pointing_9/n20181202010453_12CO_2-1_cross_OriKL_pointing/n20181202010453_12CO_2-1_cross_OriKL_pointing.fits')
            self.hdu2 = fits.open('/home/amigos/backup/radio_pointing_9/n20181202010453_12CO_2-1_cross_OriKL_pointing/n20181202010453_13CO_2-1_cross_OriKL_pointing.fits')
            self.integ_time = 10
        elif self.obs == 'edge':
            self.hdu1 = fits.open('')
            self.hdu2 = fits.open('')
            self.integ_time = 1
        elif self.obs == 'otf':
            self.hdu1 = fits.open('')
            self.hdu2 = fits.open('')
            self.integ_time = 0.6
        self.data1 = self.hdu1[1].data["DATA"]
        self.data2 = self.hdu1[1].data["DATA"]
        self.pre_dataindex()
        print(self.ind_list_hot,self.ind_list_off,self.ind_list_on)
        pass

    def pre_dataindex(self):
        sobsmode = self.hdu1[1].data['SOBSMODE']

        onmask = sobsmode == "ON"
        hotmask = sobsmode == "HOT"
        offmask = sobsmode == "OFF"

        ind_on = np.where(onmask == True)
        self.ind_list_on = [ind_on[0][i] for i in range(len(ind_on[0]))]

        ind_hot = np.where(hotmask == True)
        self.ind_list_hot = [ind_hot[0][i] for i in range(len(ind_hot[0]))]

        ind_off = np.where(offmask == True)
        self.ind_list_off = [ind_off[0][i] for i in range(len(ind_off[0]))]
        return
        
    
    def flag(self, req):
        self.timestamp = req.timestamp
        self.newdb_name = req.newdb_name
        self.obs_mode = req.obs_mode
        self.scan_num = req.scan_num
        self.lamdel = req.lamdel
        self.betdel = req.betdel
        if self.timestamp == 1:
            if self.obs_mode == 'ON':
                self.on_count += 1
            elif self.obs_mode == 'OFF':
                self.off_count += 1
            elif self.obs_mode == 'HOT':
                self.hot_count += 1
        else:
            pass
        return
    
    def stop_loop(self):
        """
        DESCRIPTION
        ===========
        This function stops data relaying loop function.
        This function is also called when Ctrl-C is pressed.

        ARGUMENTS
        =========
        Nothing.

        RETURNS
        =======
        Nothing.
        """
        self._stop_loop = True
        return

    # Master function
    # ---------------


    def pre_data(self, mode , data):
        whitenoise = np.random.randn(16384)
        _d = []
        if mode == 'ON':
            d = data[self.ind_list_on[self.on_count-1]]
            for i in range(16383):
                dd = (d[i]+d[i+1])/2
                _d.append(d[i])
                _d.append(dd+whitenoise[i])
            _d.append(d[16383])
            _d.append((d[16383]+d[0])/2)
                
        elif mode == 'HOT':
            d = data[self.ind_list_hot[self.hot_count-1]]
            for i in range(16383):
                dd = (d[i]+d[i+1])/2
                _d.append(d[i])
                _d.append(dd+whitenoise[i])
            _d.append(d[16383])
            _d.append((d[16383]+d[0])/2)
            if self.hot_count == len(self.ind_list_hot):
                self.hot_count = 0
            else:
                pass
            '''
            elif self.hot_count > len(self.ind_list_hot):
                d = data[self.ind_list_hot[len(self.ind_list_hot)-(self.hot_count)]]
                for i in range(16383):
                    dd = (d[i]+d[i+1])/2
                    _d.append(d[i])
                    _d.append(dd+whitenoise[i])
                _d.append(d[16383])
                _d.append((d[16383]+d[0])/2)
            '''
        elif mode == 'OFF':
            d = data[self.ind_list_off[self.off_count-1]]
            for i in range(16383):
                dd = (d[i]+d[i+1])/2
                _d.append(d[i])
                _d.append(dd+whitenoise[i])
            _d.append(d[16383])
            _d.append((d[16383]+d[0])/2)
        d_ = np.array(_d)
        _d = []
        return d_

    def make_data(self, obs_mode, data):
        integ_data = np.zeros(32768)
        
        if obs_mode == 'ON' and self.timestamp == 1:
            #whitenoise = numpy.random.randn(32768)
            integ_data = self.pre_data('ON', data) #+ whitedata
                
        elif obs_mode == 'OFF' and self.timestamp == 1:
            #Whitenoise = numpy.random.randn(32768)
            integ_data = self.pre_data('OFF', data) #+ whitedata
                
        elif obs_mode == 'HOT' and self.timestamp == 1:
            #whitenoise = numpy.random.randn(32768)
            integ_data = self.pre_data('HOT', data) #+ whitedata
                
        data_ = integ_data / self.sample_rate
        
        return data_


    def data_relaying_loop(self):
        """
        DESCRIPTION
        ===========
        This is the master function of this class.
        This function makes data like XFFTS's by ROS method.
        When you stop the loop, call stop_loop function.

        ARGUMENTS
        =========
        Nothing.

        RETURNS
        =======
        Nothing, but send values listed below to ROS subscriber.
        1. XFFTS_SPEC : Send spectrum data
            1. timestamp : XFFTS-format timestamp.
                Type     : str
                fmt      : '2017-10-20T09:34:13.9193PC  '
            2. BE_num    : Number of Back End Board.
                Number   : 1 - 16
                Type     : int
            3. SPEC_BE1-16 : The spectrum of each BE(1-16).
                Type       : float list
        2. XFFTS_PM : Send total power data(=continuum data).
            1. timestamp : Same as above.
            2. BE_num    : Same as above.
            3. POWER_BE1-16 : The total counts of each BE(1-16).
        """
        # Print Welcome Massage
        # ---------------------
        print('\n\n'
              '  =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n'
              '   Start : XFFTS Data Relaying Loop \n'
              '  =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-'
              '\n\n')

        # ROS setting
        # -----------
        pub = rospy.Publisher('XFFTS_SPEC', XFFTS_msg, queue_size=10)
        pub2 = rospy.Publisher('XFFTS_PM', XFFTS_pm_msg, queue_size=10)             # PM = Power Meter
        XFFTS_SPEC = XFFTS_msg()
        XFFTS_PM = XFFTS_pm_msg()

    
        # data making loop
        # ------------------
        while True:

            if self._stop_loop: break

            # get data
            # --------
            header = data_header()

            timestamp = header.timestamp

            tdatetime = datetime.strptime(timestamp, "%Y-%m-%dT%H:%M:%S.%fPC ")
            BE_num = header.BE_num
            time_int = int(tdatetime.strftime('%s'))
            time_decimal = float(tdatetime.strftime('%s.%f')) - int(tdatetime.strftime('%s'))
            print(float(tdatetime.strftime('%s.%f')), BE_num)
            
            #make data
            if self.timestamp == 1:
                spec1 = self.make_data(self.obs_mode, self.data1)
                spec2 = self.make_data(self.obs_mode, self.data2)
                pow1 = np.sum(spec1)
                pow2 = np.sum(spec2)
                XFFTS_SPEC.SPEC_BE1 = spec1
                XFFTS_SPEC.SPEC_BE2 = spec2
                XFFTS_SPEC.SPEC_BE3 = spec1
                XFFTS_SPEC.SPEC_BE4 = spec2
                XFFTS_SPEC.SPEC_BE5 = spec1
                XFFTS_SPEC.SPEC_BE6 = spec2
                XFFTS_SPEC.SPEC_BE7 = spec1
                XFFTS_SPEC.SPEC_BE8 = spec2
                XFFTS_SPEC.SPEC_BE9 = spec1
                XFFTS_SPEC.SPEC_BE10 = spec2
                XFFTS_SPEC.SPEC_BE11 = spec1
                XFFTS_SPEC.SPEC_BE12 = spec2
                XFFTS_SPEC.SPEC_BE13 = spec1
                XFFTS_SPEC.SPEC_BE14 = spec2
                XFFTS_SPEC.SPEC_BE15 = spec1
                XFFTS_SPEC.SPEC_BE16 = spec2
                XFFTS_SPEC.SPEC_BE17 = spec1
                XFFTS_SPEC.SPEC_BE18 = spec2
                XFFTS_SPEC.SPEC_BE19 = spec1
                XFFTS_SPEC.SPEC_BE20 = spec2
                
                XFFTS_PM.POWER_BE1 = pow1
                XFFTS_PM.POWER_BE2 = pow2
                XFFTS_PM.POWER_BE3 = pow1
                XFFTS_PM.POWER_BE4 = pow2
                XFFTS_PM.POWER_BE5 = pow1
                XFFTS_PM.POWER_BE6 = pow2
                XFFTS_PM.POWER_BE7 = pow1
                XFFTS_PM.POWER_BE8 = pow2
                XFFTS_PM.POWER_BE9 = pow1
                XFFTS_PM.POWER_BE10 = pow2
                XFFTS_PM.POWER_BE11 = pow1
                XFFTS_PM.POWER_BE12 = pow2
                XFFTS_PM.POWER_BE13 = pow1
                XFFTS_PM.POWER_BE14 = pow2
                XFFTS_PM.POWER_BE15 = pow1
                XFFTS_PM.POWER_BE16 = pow2
                XFFTS_PM.POWER_BE17 = pow1
                XFFTS_PM.POWER_BE18 = pow2
                XFFTS_PM.POWER_BE19 = pow1
                XFFTS_PM.POWER_BE20 = pow2
            
            else:
                spec = np.random.normal(5000, 2000, (header.BE_num, 32768))
                pow = np.sum(spec, axis=1)
                XFFTS_SPEC.SPEC_BE1 = spec[0]
                XFFTS_SPEC.SPEC_BE2 = spec[1]
                XFFTS_SPEC.SPEC_BE3 = spec[2]
                XFFTS_SPEC.SPEC_BE4 = spec[3]
                XFFTS_SPEC.SPEC_BE5 = spec[4]
                XFFTS_SPEC.SPEC_BE6 = spec[5]
                XFFTS_SPEC.SPEC_BE7 = spec[6]
                XFFTS_SPEC.SPEC_BE8 = spec[7]
                XFFTS_SPEC.SPEC_BE9 = spec[8]
                XFFTS_SPEC.SPEC_BE10 = spec[9]
                XFFTS_SPEC.SPEC_BE11 = spec[10]
                XFFTS_SPEC.SPEC_BE12 = spec[11]
                XFFTS_SPEC.SPEC_BE13 = spec[12]
                XFFTS_SPEC.SPEC_BE14 = spec[13]
                XFFTS_SPEC.SPEC_BE15 = spec[14]
                XFFTS_SPEC.SPEC_BE16 = spec[15]
                XFFTS_SPEC.SPEC_BE17 = spec[16]
                XFFTS_SPEC.SPEC_BE18 = spec[17]
                XFFTS_SPEC.SPEC_BE19 = spec[18]
                XFFTS_SPEC.SPEC_BE20 = spec[19]

                XFFTS_PM.POWER_BE1 = pow[0]
                XFFTS_PM.POWER_BE2 = pow[1]
                XFFTS_PM.POWER_BE3 = pow[2]
                XFFTS_PM.POWER_BE4 = pow[3]
                XFFTS_PM.POWER_BE5 = pow[4]
                XFFTS_PM.POWER_BE6 = pow[5]
                XFFTS_PM.POWER_BE7 = pow[6]
                XFFTS_PM.POWER_BE8 = pow[7]
                XFFTS_PM.POWER_BE9 = pow[8]
                XFFTS_PM.POWER_BE10 = pow[9]
                XFFTS_PM.POWER_BE11 = pow[10]
                XFFTS_PM.POWER_BE12 = pow[11]
                XFFTS_PM.POWER_BE13 = pow[12]
                XFFTS_PM.POWER_BE14 = pow[13]
                XFFTS_PM.POWER_BE15 = pow[14]
                XFFTS_PM.POWER_BE16 = pow[15]
                XFFTS_PM.POWER_BE17 = pow[16]
                XFFTS_PM.POWER_BE18 = pow[17]
                XFFTS_PM.POWER_BE19 = pow[18]
                XFFTS_PM.POWER_BE20 = pow[19]
                
            # ROS Data Trans
            # --------------
            # Spectrum
            XFFTS_SPEC.timestamp = time.time()
            XFFTS_SPEC.BE_num = BE_num
            pub.publish(XFFTS_SPEC)

            # total power
            XFFTS_PM.timestamp = time.time()
            XFFTS_PM.BE_num = BE_num
            pub2.publish(XFFTS_PM)

            time.sleep(self.integ_time / self.sample_rate)

        # Print Shut Down Massage
        # -----------------------
        print('\n\n'
              '  =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n'
              '   Shut Down : XFFTS Data Relaying Loop \n'
              '  =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-'
              '\n\n')
        return

    def temp_relaying_loop(self):
        """
        DESCRIPTION
        ===========
        This is the master function of this class.
        This function makes date of XFFTS Board temperature to FAC by ROS method.
        When you stop the loop, call stop_loop function.

        ARGUMENTS
        =========
        Nothing.
        
        """
        
        header = data_header()

        # ROS setting
        # -----------
        pub3 = rospy.Publisher('XFFTS_TEMP', XFFTS_temp_msg_du, queue_size=10)
        XFFTS_TEMP = XFFTS_temp_msg_du()

        while True:

            if self._stop_loop: break

            # get data
            # --------
            timestamp = time.time()
            temps = np.random.normal(150, 50, header.BE_num)
            # ROS Data Trans
            # --------------
            XFFTS_TEMP.timestamp = timestamp
            XFFTS_TEMP.TEMP_BE1 = temps[0]
            XFFTS_TEMP.TEMP_BE2 = temps[1]
            XFFTS_TEMP.TEMP_BE3 = temps[2]
            XFFTS_TEMP.TEMP_BE4 = temps[3]
            XFFTS_TEMP.TEMP_BE5 = temps[4]
            XFFTS_TEMP.TEMP_BE6 = temps[5]
            XFFTS_TEMP.TEMP_BE7 = temps[6]
            XFFTS_TEMP.TEMP_BE8 = temps[7]
            XFFTS_TEMP.TEMP_BE9 = temps[8]
            XFFTS_TEMP.TEMP_BE10 = temps[9]
            XFFTS_TEMP.TEMP_BE11 = temps[10]
            XFFTS_TEMP.TEMP_BE12 = temps[11]
            XFFTS_TEMP.TEMP_BE13 = temps[12]
            XFFTS_TEMP.TEMP_BE14 = temps[13]
            XFFTS_TEMP.TEMP_BE15 = temps[14]
            XFFTS_TEMP.TEMP_BE16 = temps[15]
            XFFTS_TEMP.TEMP_BE17 = temps[16]
            XFFTS_TEMP.TEMP_BE18 = temps[17]
            XFFTS_TEMP.TEMP_BE19 = temps[18]
            XFFTS_TEMP.TEMP_BE20 = temps[19]
            pub3.publish(XFFTS_TEMP)

            time.sleep(1)
        return

    def start_thread(self):
        th = threading.Thread(target=self.data_relaying_loop)
        th.setDaemon(True)
        th.start()

        th2 = threading.Thread(target=self.temp_relaying_loop)
        th2.setDaemon(True)
        th2.start()
        return


class data_header(object):
    header_size = 64
    def __init__(self):
        self.ieee= "EEEI"
        self.data_format = "F   "
        self.package_length = int(262224)
        self.BE_name = "XFFTS"
        self.timestamp = (datetime.now()).strftime('%Y-%m-%dT%H:%M:%S.%fPC  ')
        self.integration_time = 998993
        self.phase_number = int(1)
        self.BE_num = int(20)   #BEの数を変えれる
        self.blocking = int(1)
        return

if __name__ == '__main__':
    serv = data_server()
    # Signal handler
    # --------------
    def signal_handler(num, flame):
        serv.stop_loop()
        sys.exit()
    signal.signal(signal.SIGINT, signal_handler)

    serv.start_thread()

    while True:
        pass
