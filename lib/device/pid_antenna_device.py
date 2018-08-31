#!/usr/bin/env python3

import pyinterface
import struct
import numpy
import math
import time
import os


class antenna_device(object):

    command_az_speed = 0
    command_el_speed = 0

    #count = [0, 0]
    #target_array = [[0], [0]]

    az_rate_d = el_rate_d = 0
    pre_hensa = [0, 0]
    ihensa = [0.0, 0.0]

    enc_before = [0, 0]
    pre_arcsec = [0, 0]

    t_now = t_past = 0.0

    #PID parameter
    p_coeff = [3.7, 3.7]
    i_coeff = [3.0, 3.0]
    d_coeff = [0, 0]
    dir_name = ''
    
    def __init__(self):
        board_name = 2724
        rsw_id = 0 #rotary switch id
        self.dio = pyinterface.open(board_name, rsw_id)
        self.dio.initialize()
        ut = time.gmtime()
        self.dir_name = time.strftime("/home/necst/data/experiment/save_azel/%Y_%m_%d_%H_%M_%S", ut)
        os.mkdir(self.dir_name)
                        


    def init_speed(self):
        self.dio.output_word('OUT1_16', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#az
        self.dio.output_word('OUT17_32', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#el
        self.command_az_speed = 0
        self.command_el_speed = 0
        return

    """                                                                       
    def set_pid_param(self, param):                                            
        self.p_coeff[0] = param["az"][0]                                               self.i_coeff[0] = param["az"][1]                                       
        self.d_coeff[0] = param["az"][2]                                       
        self.p_coeff[1] = param["el"][0]                                       
        self.i_coeff[1] = param["el"][1]                                               self.d_coeff[1] = param["el"][2]                                               return                                                                 
    """

    def move_azel(self, az_arcsec, el_arcsec, enc_az, enc_el, pid_param=None, m_bStop = 'FALSE'):
        MOTOR_MAXSTEP = 1000
        MOTOR_AZ_MAXRATE = 10000
        MOTOR_EL_MAXRATE = 12000

        #self.set_pid_param(pid_param)

        #for az >= 180*3600 and az <= -180*3600
        if enc_az > 40*3600 and az_arcsec+360*3600 < 220*3600:
            az_arcsec += 360*3600
        elif enc_az < -40*3600 and az_arcsec-360*3600 > -220*3600:
            az_arcsec -= 360*3600
            
        if self.t_past == 0.0:
            self.t_past = time.time()
        else:
            pass
        self.t_now = time.time()

        ret_az = calc_pid(az_arcsec, enc_az, self.pre_arcsec[0], self.pre_hensa[0], self.ihensa[0], self.enc_before[0], self.t_now, self.t_past, self.p_coeff[0], self.i_coeff[0], self.d_coeff[0])
        az_rate_ref = ret_az[0]
        ret_el = calc_pid(el_arcsec, enc_el, self.pre_arcsec[1], self.pre_hensa[1], self.ihensa[1], self.enc_before[1], self.t_now, self.t_past, self.p_coeff[1], self.i_coeff[1], self.d_coeff[1])
        el_rate_ref = ret_el[0]

        save_to = os.path.join(self.dir_name, 'pid.txt')
        f = open(save_to ,'a')
        f.write('%s %s %s %s %s %s %s\n'%(str(self.t_now),str(self.p_coeff[0]*(az_arcsec-enc_az)),str(self.i_coeff[0]*ret_az[1]*(self.t_now+self.t_past)),str(self.d_coeff[0]*(az_arcsec-enc_az-self.pre_hensa[0])/(self.t_now-self.t_past)),str(self.p_coeff[1]*(el_arcsec-enc_el)),str(self.i_coeff[1]*ret_el[1]*(self.t_now+self.t_past)),str(self.d_coeff[1]*(el_arcsec-enc_el-self.pre_hensa[1])/(self.t_now-self.t_past))))
        f.close()
        
        #update
        self.enc_before = [enc_az, enc_el]
        self.pre_hensa = [az_arcsec - enc_az, el_arcsec - enc_el]
        self.pre_arcsec = [az_arcsec, el_arcsec]
        self.ihensa = [ret_az[1], ret_el[1]]
        self.t_past = self.t_now

        #limit of acc         
        if abs(az_rate_ref - self.az_rate_d) < MOTOR_MAXSTEP:
            self.az_rate_d = az_rate_ref
        else:
            if (az_rate_ref - self.az_rate_d) < 0:
                a = -1
            else:
                a = 1
            self.az_rate_d += a*MOTOR_MAXSTEP
        if abs(el_rate_ref - self.el_rate_d) < MOTOR_MAXSTEP:
            self.el_rate_d = el_rate_ref
        else:
            if (el_rate_ref - self.el_rate_d) < 0:
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
        scaling_dummy = int(self.az_rate_d * 7/12 * 10000/3600)
        dummy_byte = list(map(int,  ''.join([format(b, '08b')[::-1] for b in struct.pack('<h', scaling_dummy)])))
        self.dio.output_word('OUT1_16', dummy_byte)
        self.az_rate_d = dummy

        if m_bStop == 'TRUE':
            dummy = 0
        else:
            dummy = int(self.el_rate_d)
            
        self.command_el_speed = dummy
        scaling_dummy = int(self.el_rate_d * 7/12 * 10000/3600)
        dummy_byte = list(map(int,  ''.join([format(b, '08b')[::-1] for b in struct.pack('<h', scaling_dummy)])))
        self.dio.output_word('OUT17_32', dummy_byte)
        self.el_rate_d = dummy
         

        return [ret_az[0], ret_el[0]]
    
    """                                                                         
    def medi_calc(self, target_speed, i):                                      
        target_num = 13 # number of median array                               
        self.target_array[i].insert(0, target_speed)                           
        if self.count[i] < target_num:                                         
            self.count[i] += 1
        else:                                                                 
            self.target_array[i].pop(13)                                       
                                                                             
        median = numpy.median(self.target_array[i])                            
        return median                                                          
    """

    def emergency_stop(self):
        for i in range(5):
            self.dio.output_word('OUT1_16', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
            self.dio.output_word('OUT17_32', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
            time.sleep(0.05)
            self.command_az_speed = 0
            self.command_el_speed = 0
            return

def calc_pid(target_arcsec, encoder_arcsec, pre_arcsec, pre_hensa, ihensa, enc_before, t_now, t_past, p_coeff, i_coeff, d_coeff):
    """                                                                         
    DESCRIPTION                                                                 
    ===========                                                                 
    This function determine az&el speed for antenna                             
    """

    #calculate ichi_hensa
    hensa = target_arcsec - encoder_arcsec

    dhensa = hensa - pre_hensa
    if math.fabs(dhensa) > 1:
        dhensa = 0

    if (encoder_arcsec - enc_before) != 0.0:
        current_speed = (encoder_arcsec - enc_before) / (t_now-t_past)
            
    if pre_arcsec == 0: # for first move
        target_speed = 0
    else:
        target_speed = (target_arcsec - pre_arcsec)/(t_now - t_past)

    ihensa += (hensa + pre_hensa)/2
    if math.fabs(hensa) > 50:
        ihensa = 0.0

        #PID
    rate = target_speed + p_coeff*hensa + i_coeff*ihensa*(t_now-t_past) + d_coeff*dhensa/(t_now-t_past)
    
    return [rate, ihensa]
