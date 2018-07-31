#!/usr/bin/env python3

import pyinterface
import struct
import numpy
import math

class antenna_device(object):

    command_az_speed = 0
    command_el_speed = 0
    
    count = 0
    az_array = []
    el_array = []
    target_az_array = []
    target_el_array = []

    az_rate = el_rate = 0
    az_rate_d = el_rate_d = 0
    pre_hensa_az = pre_hensa_el = 0
    ihensa_az = ihensa_el = 0.0
    
    az_enc_before = el_enc_before = 0
    az_err_before = el_err_before = 0
    current_speed_az = current_speed_el = 0.0
    pre_az_arcsec = pre_el_arcsec = 0

    t1 = t2 =0.0
    t_az = t_el = 0.0

    #PID parameter
    p_az_coeff = 3.7
    i_az_coeff = 3.0
    d_az_coeff = 0
    s_az_coeff = 0
    p_el_coeff = 3.7
    i_el_coeff = 3.0
    d_el_coeff = 0
    s_el_coeff = 0


    def __init__(self):
        board_name = 2724
        rsw_id = 0 #rotary switch id
        self.dio = pyinterface.open(board_name, rsw_id)
        self.dio.initialize()


    def init_speed(self):
        self.dio.output_word('OUT1_16', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#az
        self.dio.output_word('OUT17_32', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#el
        self.command_az_speed = 0
        self.command_el_speed = 0
        return

    def move_azel(self, az_arcsec, el_arcsec, enc_az, enc_el, az_max_rate = 10000, el_max_rate = 12000, m_bStop = 'FALSE'):
        MOTOR_MAXSTEP = 1000
        MOTOR_AZ_MAXRATE = 10000
        MOTOR_EL_MAXRATE = 12000
        
        ret = self.calc_pid(az_arcsec, el_arcsec, enc_az, enc_el, az_max_rate, el_max_rate)
        az_rate_ref = ret[0]
        el_rate_ref = ret[1]
        
        # command value to target value
        daz_rate = az_rate_ref - self.az_rate_d
        del_rate = el_rate_ref - self.el_rate_d
        
        #limit of acc
        if abs(daz_rate) < MOTOR_MAXSTEP:
            self.az_rate_d = az_rate_ref
        else:
            if daz_rate < 0:
                a = -1
            else:
                a = 1
            self.az_rate_d += a*MOTOR_MAXSTEP
        if abs(del_rate) < MOTOR_MAXSTEP:
            self.el_rate_d = el_rate_ref
        else:
            if del_rate < 0:
                a = -1
            else:
                a = 1
            self.el_rate_d += a*MOTOR_MAXSTEP
        
        #limit of max v
        if self.az_rate_d > MOTOR_AZ_MAXRATE:
            self.az_rate_d = MOTOR_AZ_MAXRATE
        if self.az_rate_d < -MOTOR_AZ_MAXRATE:
            self.az_rate_d = -MOTOR_AZ_MAXRATE
        if self.el_rate_d > MOTOR_EL_MAXRATE:
            self.el_rate_d = MOTOR_EL_MAXRATE
        if self.el_rate_d < -MOTOR_EL_MAXRATE:
            self.el_rate_d = -MOTOR_EL_MAXRATE
        
        # output to port
        if m_bStop == 'TRUE':
            dummy = 0
        else:
            dummy = int(self.az_rate_d)
        
        self.command_az_speed = dummy
        dummy_byte = list(map(int,  ''.join([format(b, '08b')[::-1] for b in struct.pack('<h', dummy)])))
        self.dio.output_word('OUT1_16', dummy_byte)
        self.az_rate_d = dummy
        
        if m_bStop == 'TRUE':
            dummy = 0
        else:
            dummy = int(self.el_rate_d)
        
        self.command_el_speed = dummy
        dummy_byte = list(map(int,  ''.join([format(b, '08b')[::-1] for b in struct.pack('<h', dummy)])))
        self.dio.output_word('OUT17_32', dummy_byte)
        self.el_rate_d = dummy
        
        return ret

    def calc_pid(self, az_arcsec, el_arcsec, az_enc, el_enc, az_max_rate, el_max_rate):
        """
        DESCRIPTION
        ===========
        This function determine az&el speed for antenna 
        """
        DEG2ARCSEC = 3600.
        if self.t2 == 0.0:
         self.t2 = time.time()
        else:
            pass

        #ROS_version
        #-----------
        #for az >= 180*3600 and az <= -180*3600
        if az_enc > 40*3600 and az_arcsec+360*3600 < 220*3600:
            az_arcsec += 360*3600
        elif az_enc < -40*3600 and az_arcsec-360*3600 > -220*3600:
            az_arcsec -= 360*3600
        
        #calculate ichi_hensa
        az_err = az_arcsec-az_enc
        el_err = el_arcsec-el_enc

        hensa_az = az_arcsec - az_enc
        hensa_el = el_arcsec - el_enc
        
        dhensa_az = hensa_az - self.pre_hensa_az
        if math.fabs(dhensa_az) > 1:
            dhensa_az = 0
        dhensa_el = hensa_el - self.pre_hensa_el
        if math.fabs(dhensa_el) > 1:
            dhensa_el = 0
        
        self.t1 = time.time()
        if self.t_az == 0.0 and self.t_el == 0.0:
            self.t_az = self.t_el = self.t1
        else:
            if (az_enc - self.az_enc_before) != 0.0:
                self.current_speed_az = (az_enc - self.az_enc_before) / (self.t1-self.t_az)
                self.t_az = self.t1
            if (el_enc - self.el_enc_before) != 0.0:
                self.current_speed_el = (el_enc - self.el_enc_before) / (self.t1-self.t_el)
                self.t_el = self.t1
        
        if self.pre_az_arcsec == 0: # for first move
            target_speed_az = 0
        else:
            target_speed_az = (az_arcsec-self.pre_az_arcsec)/(self.t1-self.t2)
        if self.pre_el_arcsec == 0: # for first move
            target_speed_el = 0
        else:
            target_speed_el = (el_arcsec-self.pre_el_arcsec)/(self.t1-self.t2)
        
        ret = self.medi_calc(target_speed_az, target_speed_el)
        target_speed_az = ret[0]
        target_speed_el = ret[1]
        
        self.ihensa_az += (hensa_az + self.pre_hensa_az)/2
        self.ihensa_el += (hensa_el + self.pre_hensa_el)/2
        if math.fabs(hensa_az) > 50:
            self.ihensa_az = 0
        if math.fabs(hensa_el) > 50:
            self.ihensa_el = 0
        
        
        self.az_rate = target_speed_az + (self.current_speed_az - self.az_rate) * self.s_az_coeff + self.p_az_coeff*hensa_az + self.i_az_coeff*self.ihensa_az*(self.t1-self.t2) + self.d_az_coeff*dhensa_az/(self.t1-self.t2)
        self.el_rate = target_speed_el + (self.current_speed_el - self.el_rate) * self.s_el_coeff + self.p_el_coeff*hensa_el + self.i_el_coeff*self.ihensa_el*(self.t1-self.t2) + self.d_el_coeff*dhensa_el/(self.t1-self.t2)
        
        
        #update
        self.az_enc_before = az_enc
        self.el_enc_before = el_enc
        self.az_err_before = az_err
        self.el_err_before = el_err
        
        self.pre_hensa_az = hensa_az
        self.pre_hensa_el = hensa_el
        
        self.pre_az_arcsec = az_arcsec
        self.pre_el_arcsec = el_arcsec
        
        self.t2 = self.t1
        
        az_rate_ref = int(self.az_rate) #??
        el_rate_ref = int(self.el_rate) #??
        return [az_rate_ref, el_rate_ref]

    def medi_calc(self, target_az, target_el):
        target_num = 13 # number of median array
        if self.count < target_num:
            self.target_az_array.insert(0, target_az)
            self.target_el_array.insert(0, target_el)
            self.count += 1
        else:
            self.target_az_array.insert(0, target_az)
            self.target_el_array.insert(0, target_el)
            self.target_az_array.pop(13)
            self.target_el_array.pop(13)
        
        median_az = numpy.median(self.target_az_array)
        median_el = numpy.median(self.target_el_array)
        return [median_az, median_el]

