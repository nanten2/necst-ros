
#!/usr/bin/env python3

#import pyinterface
import struct
import numpy
import math
import time

class antenna_device(object):

    command_az_speed = 0
    command_el_speed = 0
    
    count = [0, 0]
    target_array = [[0], [0]]

    rate = [0, 0]
    az_rate_d = el_rate_d = 0
    pre_hensa = [0, 0]
    ihensa = [0.0, 0.0]
    
    enc_before = [0, 0]
    err_before = [0, 0]
    current_speed = [0.0, 0.0]
    pre_arcsec = [0, 0]

    t_now = t_past = 0.0
    t = [0.0, 0.0]

    #PID parameter
    p_coeff = [3.7, 3.7]
    i_coeff = [3.0, 3.0]
    d_coeff = [0, 0]


    def __init__(self):
        board_name = 2724
        rsw_id = 0 #rotary switch id
        #self.dio = pyinterface.open(board_name, rsw_id)
        #self.dio.initialize()


    def init_speed(self):
        #self.dio.output_word('OUT1_16', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#az
        #self.dio.output_word('OUT17_32', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#el
        self.command_az_speed = 0
        self.command_el_speed = 0
        return

    """
    def set_pid_param(self, param):
        self.p_coeff[0] = param["az"][0]
        self.i_coeff[0] = param["az"][1]
        self.d_coeff[0] = param["az"][2]
        self.p_coeff[1] = param["el"][0]
        self.i_coeff[1] = param["el"][1]
        self.d_coeff[1] = param["el"][2]
        return
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

        ret_az = self.calc_pid(az_arcsec, enc_az, mode="az")
        az_rate_ref = ret_az
        ret_el = self.calc_pid(el_arcsec, enc_el, mode="el")
        el_rate_ref = ret_el
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
        #self.dio.output_word('OUT1_16', dummy_byte)
        self.az_rate_d = dummy
        
        if m_bStop == 'TRUE':
            dummy = 0
        else:
            dummy = int(self.el_rate_d)
        
        self.command_el_speed = dummy
        scaling_dummy = int(self.el_rate_d * 7/12 * 10000/3600)
        dummy_byte = list(map(int,  ''.join([format(b, '08b')[::-1] for b in struct.pack('<h', scaling_dummy)])))
        #self.dio.output_word('OUT17_32', dummy_byte)
        self.el_rate_d = dummy
        
        return [ret_az, ret_el]

    def calc_pid(self, target_arcsec, encoder_arcsec, mode):
        """
        DESCRIPTION
        ===========
        This function determine az&el speed for antenna 
        """
        DEG2ARCSEC = 3600.
        if mode == "az":
            i = 0
        elif mode == "el":
            i = 1
        else:
            return

        
        #calculate ichi_hensa
        err = target_arcsec - encoder_arcsec

        hensa = target_arcsec - encoder_arcsec 
        
        dhensa = hensa - self.pre_hensa[i]
        if math.fabs(dhensa) > 1:
            dhensa = 0
        
        if self.t[i] == 0.0:
            self.t[i] = self.t_now
        else:
            if (encoder_arcsec - self.enc_before[i]) != 0.0:
                self.current_speed[i] = (encoder_arcsec - self.enc_before[i]) / (self.t_now-self.t[i])
        
        if self.pre_arcsec[i] == 0: # for first move
            target_speed = 0
        else:
            target_speed = (target_arcsec - self.pre_arcsec[i])/(self.t_now - self.t_past)
        
        ret = self.medi_calc(target_speed, i)
        target_speed = ret
        
        self.ihensa[i] += (hensa + self.pre_hensa[i])/2
        if math.fabs(hensa) > 50:
            self.ihensa[i] = 0.0
        
        #PID
        self.rate[i] = target_speed + self.p_coeff[i]*hensa + self.i_coeff[i]*self.ihensa[i]*(self.t_now-self.t_past) + self.d_coeff[i]*dhensa/(self.t_now-self.t_past)
        
        #update
        self.enc_before[i] = encoder_arcsec
        self.err_before[i] = err
        self.pre_hensa[i] = hensa
        self.pre_arcsec[i] = target_arcsec
        self.t[i] = self.t_now
        
        return self.rate[i]

    def medi_calc(self, target_speed, i):
        target_num = 13 # number of median array
        self.target_array[i].insert(0, target_speed)
        if self.count[i] < target_num:
            self.count[i] += 1
        else:
            self.target_array[i].pop(13)
        
        median = numpy.median(self.target_array[i])
        return median

    def emergency_stop(self):
        for i in range(5):
            #self.dev.dio.output_word('OUT1_16', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
            #self.dev.dio.output_word('OUT17_32', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
            time.sleep(0.05)
        return
