#!/usr/bin/env python3
"""
history
2018/1/15 edited by Shiotani
ROS_antenna_move.py
"""
import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import rospy
import numpy
import time
import threading
import math
sys.path.append("/home/necst/ros/src/necst/lib")
import numpy as np
import struct
import pyinterface

#ROS/import field
#----------------
from necst.msg import Status_antenna_msg
from necst.msg import list_azelmsg
from necst.msg import Status_encoder_msg
from std_msgs.msg import String
from std_msgs.msg import Bool

class antenna_move(object):
    
    #initial parameter
    #-----------------
    parameters = {
        'az_list':[0]*300,
        'el_list':[0]*300,
        'start_time':0,
        'flag':0
        }
    enc_parameter = {
        'az_enc':0,
        'el_enc':0
        }
    B_time = 1

    stop_flag = 0
    task = 0
    error = False #False = ok
    emergency_flag = False
    limit_flag = 0###(0/1=okay/NG)
    limit_az = True###(True/False = okay/limit)
    limit_el = True
    command_az = 0
    command_el = 0
    current_az = 0
    current_el = 0

    command_az_speed = 0
    command_el_speed = 0
    """
    ###for module
    az_rate_d = 0
    el_rate_d = 0

    m_stop_rate_az = 0
    m_stop_rate_el = 0
    """

    MOTOR_SAMPLING = 10 #memo
    dt = MOTOR_SAMPLING/1000.
    
    count = 0
    az_array = []
    el_array = []
    target_az_array = []
    target_el_array = []

    #error_box = [0]*32###0921
    #azv_before = elv_before = 0
    az_enc_before = el_enc_before = 0
    az_rate = el_rate = 0
    az_rate_d = el_rate_d = 0
    m_stop_rate_az = m_stop_rate_el = 0
    t1 = t2 = 0.0
    t_az = t_el = 0.0
    current_speed_az = current_speed_el = 0.0
    pre_hensa_az = pre_hensa_el = 0
    ihensa_az = ihensa_el = 0.0
    pre_az_arcsec = pre_el_arcsec = 0
    
    indaz = 0
    indel = 0
    
    az_encmoni = 0
    el_encmoni = 0
    az_targetmoni = 0
    el_targetmoni = 0
    az_targetspeedmoni = 0
    el_targetspeedmoni = 0
    az_hensamoni = 0
    el_hensamoni = 0
    az_ihensamoni = 0
    el_ihensamoni = 0
    
    #for drive.py
    t1_moni = 0
    t2_moni = 0
    az_pidihensamoni = 0
    el_pidihensamoni = 0
    
    
    
    th_az = 0
    th_el = 0
    server_flag = []
    time_list = [0,0,0,0,0]
    save_time = 0
    end_flag = 0

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
        self.node_status = 'node_start[ROS_antenna_move]'
        pass
    
    def start_thread(self):
        th = threading.Thread(target = self.act_azel)
        th.setDaemon(True)
        th.start()
        th2 = threading.Thread(target = self.pub_status)
        th2.setDaemon(True)
        th2.start()

    def set_parameter(self, req):
        """
        DESCRIPTION
        ===========
        This function recieves azel_list and start_time from publisher in ROS_antenna.py 
        """
        print('set_parameter')
        self.parameters['az_list'] = req.az_list
        self.parameters['el_list'] = req.el_list
        self.parameters['start_time'] = req.start_time
        """
        if not self.limit_check():
            self.stop_flag = 1
            return
        """
        print('start_time : ', self.parameters['start_time'])
        self.stop_flag = 0
        return

    def set_enc_parameter(self, req):
        """
        DESCRIPTION
        ===========
        This function recieves encoder parameter(antenna's Az and El) from publisher in ROS_encoder.py
        This parameter is needed for PID control
        """
        self.enc_parameter['az_enc'] = req.enc_az
        self.enc_parameter['el_enc'] = req.enc_el 
        return

    def limit_check(self):
        """
        DESCRIPTION
        ===========
        This function checks limit azel_list 
        """
        for i in range(len(self.parameters['az_list'])):
            #print(self.parameters['az_list'][i],self.parameters['el_list'][i])
            if self.parameters['az_list'][i] >= 280*3600 or  self.parameters['az_list'][i] <=-280*3600:#kari
                rospy.logwarn('!!!limit az!!!')
                rospy.logwarn(self.parameters['az_list'][i])
                self.limit_flag = False
                self.error = True
                return False
            
            if self.parameters['el_list'][i] >= 89*3600 or  self.parameters['el_list'][i] <= 0*3600:#kari
                rospy.logwarn('!!!limit el!!!')
                rospy.logwarn(self.parameters['el_list'][i])
                self.limit_flag = False
                self.error = True
                return False
            else:
                return True

    def comp(self):
        """
        DESCRIPTION
        ===========
        This function determine target Az and El from azel_list
        """
        
        n = len(self.parameters['az_list'])
        st = self.parameters['start_time']
        ct = time.time()
        st_e = float(st) + float(n*0.1)#0.1 = interval
        print(n, st, ct, st_e, 'n, st, ct ,st_e')

        #time check
        #----------
        
        if st - ct >=0:
            print(st - ct,' [sec] waiting...')
            #print('wait starting azel list or send another list')
            #time.sleep(st-ct)
            return

        if ct - st_e >=0:
            rospy.loginfo('!!!azel_list is end!!!')
            self.stop_flag = 1
            for i in range(5):
                self.dio.output_word('OUT1_16', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#az
                self.dio.output_word('OUT17_32',[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#el
                self.command_az_speed = 0
                self.command_el_speed = 0
                time.sleep(0.25)
            return

        else:
            for i in range(len(self.parameters['az_list'])):
                st2 = st + (i*0.1)
                num = i
                if st2 - ct >0:
                    #num = i
                    break
            if num + 1 == len((self.parameters['az_list'])):
                return                    
            x1 = self.parameters['az_list'][num]
            x2 = self.parameters['az_list'][num+1]
            y1 = self.parameters['el_list'][num]
            y2 = self.parameters['el_list'][num+1]
            rospy.loginfo('send comp azel')
            print(x1,x2,y1,y2,st2)
            return (x1,x2,y1,y2,st2)

    def act_azel(self):
        while True:
            if self.stop_flag:
                print('stop_flag ON')
                self.dio.output_word('OUT1_16', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#az
                self.dio.output_word('OUT17_32',[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#el
                self.command_az_speed = 0
                self.command_el_speed = 0
                time.sleep(1)
                continue

            b_time2 = time.time()
            ret = self.comp()
            a_time2=time.time()
            if ret == None:
                time.sleep(0.1)
                continue
            else:
                b_time3 = time.time()
                az = ret[1] - ret[0]
                el = ret[3] - ret[2]
                c = time.time()
                st = ret[4]
                tar_az = ret[0] + az*(c-st)
                tar_el = ret[2] + el*(c-st)
                #2nd limit check (1st limit check is in ROS_antenna.py)
                if tar_az > 240*3600. or tar_el < -240*3600.:
                    self.stop_flag = False
                    print('!!!target az limit!!! : ', tar_az)
                    continue
                if tar_el > 89*3600. or tar_el < 0:
                    self.stop_flag = False
                    print('!!!target el limit!!! : ', tar_el)
                    continue
                self.command_az = tar_az
                self.command_el = tar_el
                d_t = st - c
                a_time3=time.time()
                print(az, el, c, st, tar_az, tar_el,"####az,el,c,st,tar_az,tar_el")
                #print(a_time3-b_time3,'check#%#%')
                #rospy.loginfo(d_t)
                #print(d_t)
                #print(type(d_t))
                #time.sleep(d_t)###for check
                if self.emergency_flag:
                    time.sleep(0.1)
                    continue
                #self.move_azel(tar_az,tar_el,10000,12000)
                self.azel_move(tar_az,tar_el,10000,12000)
                time.sleep(0.01)
           


#module part(this part is copy from nanten_main_controller.py(necst ver1.0 script))
#----------------------------------------------------------------------------------

    """
    def __init__(self):
        self.dio = pyinterface.create_gpg2000(3)
        self.enc = antenna_enc.enc_monitor_client('172.20.0.11',8002)
        #self.enc = antenna_enc.enc_controller()
        ret = self.enc.read_azel()
        self.az_encmoni = ret[0]
        self.el_encmoni = ret[1]
        pass
    """
    
    """#not need 0921

    def read_enc(self):
        t1 = time.time()
        ret = self.enc.read_azel()
        t2 = time.time()
        self.th_az = ret[0]
        self.th_el = ret[1]
        
        if t2 - t1 <= 0.005:
            self.time_list[0] = self.time_list[0] + 1
        elif t2 - t1 <= 0.01:
            self.time_list[1] = self.time_list[1] + 1
        elif t2 - t1 <= 0.015:
            self.time_list[2] = self.time_list[2] + 1
        elif t2 - t1 <= 0.02:
            self.time_list[3] = self.time_list[3] + 1
        elif t2 - t1 <= 0.025:
            self.time_list[4] = self.time_list[4] + 1
        return
        
    """

    
    def count_time(self):#not in use
        flag_list = self.server_flag[:]
        time.sleep(1)
        if self.server_flag == flag_list:
            self.dio.output_word('OUT1_16', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
            self.sdio.output_word('OUT17_32', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
            self.end_flag = 1
            for i in range(1000):
                print(flag_list)
                return
        
        
        if self.save_time == 0:
            self.save_time = time.time()
        tv = time.time()
        if tv - self.save_time >= 60.:
            name = "enc_hist"+str(int(tv))+".txt"
            f = open(name, "w")
            for i in range(5):
                f.write(str(self.time_list[i])+"\n")
            f.close()
            self.save_time = time.time()
            self.time_list = [0, 0, 0, 0, 0]
        
        return    
    
    
    def init_speed(self):
        self.dio.output_word('OUT1_16', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#az
        self.dio.output_word('OUT17_32', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#el
        self.command_az_speed = 0
        self.command_el_speed = 0
        return
    

    #ROS_version
    #cur_az cur_el means encoder_azel
    def azel_move(self, az_arcsec, el_arcsec, az_max_rate, el_max_rate):
        test_flag = 1

        self.indaz = az_arcsec
        self.indel = el_arcsec
        
        self.enc_az = self.enc_parameter['az_enc']
        self.enc_el = self.enc_parameter['el_enc']
            
        if abs(az_arcsec - self.enc_az) >= 1 or abs(el_arcsec - self.enc_el) > 1:###self.enc_az is provisonal
            b_time = time.time()
            self.move_azel(az_arcsec, el_arcsec, az_max_rate, el_max_rate)
                
            interval = time.time()-b_time
            if interval <= 0.01:
                time.sleep(0.01-interval)
                    
            else:
                self.dio.output_word('OUT1_16', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#az
                self.dio.output_word('OUT17_32', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#el
                self.command_az_speed = 0
                self.command_el_speed = 0
                return 0
                
            return 1
            
            
    def move_azel(self, az_arcsec, el_arcsec, az_max_rate = 16000, el_max_rate = 12000, m_bStop = 'FALSE'):
        MOTOR_MAXSTEP = 1000
        #MOTOR_AZ_MAXRATE = 16000
        MOTOR_AZ_MAXRATE = 10000
        MOTOR_EL_MAXRATE = 12000
        stop_flag = 0
        
        ret = self.calc_pid(az_arcsec, el_arcsec, az_max_rate, el_max_rate)
        rospy.loginfo(ret)
        az_rate_ref = ret[0]
        el_rate_ref = ret[1]
        Az_track_flag = ret[2]
        El_track_flag = ret[3]
        
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
        
        # confirm limit of controll rack => forced outage
        #if(0< motordrv_nanten2_cw_limit()+motordrv_nanten2_ccw_limit()+motordrv_nanten2_up_limit()+motordrv_nanten2_down_limit())
        #     motordrv_nanten2_drive_on(FALSE,FALSE);
        
        
        # output to port
        if m_bStop == 'TRUE':
            dummy = self.m_stop_rate_az
        else:
            dummy = int(self.az_rate_d)
        
        
        #for 1st limit
        """NEED but error 0922
        ret = self.dio.ctrl.in_byte("FBIDIO_IN1_8")

        if (ret>>2 & 0x01) == 1:
            pass
        else:
            dummy = 0
            stop_flag = 1
        """
        
        #dummy=m_bStop==TRUE?m_stop_rate_az:motor_param.az_rate_ref;
        #self.dio.ctrl.out_word("FBIDIO_OUT1_16", dummy)#0922
        #print(dummy)
        #dummy_byte = bin(dummy)
        #dummy_byte = dummy_byte[2:]
        #dummy_byte = list(map(int,dummy_byte))
        #dummy_byte = dummy_byte[::-1]
        #_len = len(dummy_byte)
        #for i in range(16 - _len):
        #    dummy_byte.append(0)
        self.command_az_speed = dummy
        dummy_byte = list(map(int,  ''.join([format(b, '08b')[::-1] for b in struct.pack('<h', dummy)])))
        self.dio.output_word('OUT1_16', dummy_byte)
        #dioOutputWord(CONTROLER_BASE2,0x00,dummy)  output port is unreliable
        self.az_rate_d = dummy
        
        if m_bStop == 'TRUE':
            dummy = self.m_stop_rate_el
        else:
            dummy = int(self.el_rate_d)
        """NEED but error 0922
        if (ret>>3 & 0x01) == 1:
            pass
        else:
            dummy = 0
            stop_flag = 1
        """
        
        #dummy=m_bStop==TRUE?m_stop_rate_el:motor_param.el_rate_ref;
        #self.dio.ctrl.out_word("FBIDIO_OUT17_32", dummy)#0921
        #print(dummy)
        self.command_el_speed = dummy
        dummy_byte = list(map(int,  ''.join([format(b, '08b')[::-1] for b in struct.pack('<h', dummy)])))

        #dummy_byte = bin(dummy)
        #dummy_byte = dummy_byte[2:]
        #dummy_byte = list(map(int,dummy_byte))
        #dummy_byte = dummy_byte[::-1]
        #_len = len(dummy_byte)
        #for i in range(16 - _len):
        #    dummy_byte.append(0)
        self.dio.output_word('OUT17_32', dummy_byte)
        #diOutputWord(CONTROLER_BASE2,0x02,dummy);
        self.el_rate_d = dummy
        
        if stop_flag:
            rospy.logwarn('')
            sys.exit()
        print('move check')
        return [Az_track_flag, El_track_flag]


    def calc_pid(self, az_arcsec, el_arcsec, az_max_rate, el_max_rate):
        """
        DESCRIPTION
        ===========
        This function determine az&el speed for antenna 
        """
        #p_az_coeff = 3.7
        #i_az_coeff = 3.0
        #d_az_coeff = 0
        #s_az_coeff = 0
        #p_el_coeff = 3.7
        #i_el_coeff = 3.0
        #d_el_coeff = 0
        #s_el_coeff = 0
        
        DEG2ARCSEC = 3600.
        m_bAzTrack = "FALSE"
        m_bElTrack = "FALSE"
        if self.t2 == 0.0:
         self.t2 = time.time()
        else:
            pass
        
        
        
        tv = time.time()
        self.server_flag.append(tv)
        if len(self.server_flag) == 4:
            self.server_flag.pop(0)
        self.end_flag = 0
        
        """not need? 0921
        enc_thread = threading.Thread(target = self.read_enc)
        enc_thread.start()
        count_thread = threading.Thread(target = self.count_time)
        count_thread.start()
        enc_thread.join()
        """
        
        #"""not need tabun
        if self.end_flag:
            for i in range(2000):
                print("!!!end!!!")
            sys.exit()
        #"""
        
        #ret = self.enc.read_azel()
        #az_enc = ret[0]
        #el_enc = ret[1]
            
        """
        #older version
        #-------------
        az_enc = self.th_az
        el_enc = self.th_el
        """

        #ROS_version
        #-----------
        az_enc = self.enc_parameter['az_enc']
        el_enc = self.enc_parameter['el_enc']

        #for az >= 180*3600 and az <= -180*3600
        if az_enc > 40*3600 and az_arcsec+360*3600 < 220*3600:
            az_arcsec += 360*3600
        elif az_enc < -40*3600 and az_arcsec-360*3600 > -220*3600:
            az_arcsec -= 360*3600
        
        #self.az_encmoni = ret[0]
        #self.el_encmoni = ret[1]
        self.az_encmoni = self.th_az
        self.el_encmoni = self.th_el
        self.az_targetmoni = az_arcsec
        self.el_targetmoni = el_arcsec
        
        #calculate ichi_hensa
        az_err = az_arcsec-az_enc
        el_err = el_arcsec-el_enc
        
        """
        #old ver(Unknown)
        #integrate error
        az_err_integral_integral+=az_err_integral*dt;
        el_err_integral_integral+=el_err_integral*dt;
        
        #derivative
        azv_err = azv-(az_enc-self.az_enc_before)/self.dt
        elv_err = elv-(el_enc-self.el_enc_before)/self.dt
        """
        
        """
        azv_acc = (azv-self.azv_before)
        elv_acc = (elv-self.elv_before)
        self.azv_before = azv
        self.elv_before = elv
        
        if azv_acc > 50:
            azv_acc = 50
        elif azv_acc < -50:
            azv_acc = -50
        
        if elv_acc > 50:
            elv_acc = 50
        elif elv_acc < -50:
            elv_acc = -50
        """
        
        target_az = az_arcsec
        target_el = el_arcsec
        hensa_az = target_az - az_enc
        hensa_el = target_el - el_enc
        ret = self.ave_calc(hensa_az, hensa_el) #average of hensa_az(ret[0]) and hensa_el(ret[1])
        
        if 3 > math.fabs(ret[0]):
            m_bAzTrack = "TRUE" #def Paz=2?
        else:
            # az_err_integral += (self.az_err_before+az_err)*self.dt/2.+azv_acc*0.0
            m_bAzTrack = 'FALSE'
            pass
        if 3 > math.fabs(ret[1]):
            m_bElTrack = "TRUE" #def Pel=2?
        else:
            #el_err_integral += (self.el_err_before+el_err)*self.dt/2.+elv_acc*0.0
            m_bElTrack = 'FALSE'
            pass
        
        
        
        self.az_hensamoni = hensa_az
        self.el_hensamoni = hensa_el
        
        dhensa_az = hensa_az - self.pre_hensa_az
        dhensa_el = hensa_el - self.pre_hensa_el
        if math.fabs(dhensa_az) > 1:
            dhensa_az = 0
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
        
        self.ihensa_az += (hensa_az+self.pre_hensa_az)/2
        self.ihensa_el += (hensa_el+self.pre_hensa_el)/2
        #if math.fabs(hensa_az) > math.fabs(self.current_speed_az)/10.+10.:
        #self.ihensa_az = 0
        #if math.fabs(hensa_el) > math.fabs(self.current_speed_el)/10.+10.:
            #self.ihensa_el = 0
        """
        if math.fabs(hensa_az) > 150:
            self.ihensa_az = 0
        if math.fabs(hensa_el) > 150:
            self.ihensa_el = 0
        
        if math.fabs(hensa_az) > 100:
            self.ihensa_az = 0
        if math.fabs(hensa_el) > 100:
            self.ihensa_el = 0
        """
        if math.fabs(hensa_az) > 50:
            self.ihensa_az = 0
        if math.fabs(hensa_el) > 50:
            self.ihensa_el = 0
        
        """Previous
        if math.fabs(hensa_az) >= 0.00: # don't use ihensa?
            ihensa_az = 0
            #hensa_flag_az = 0;
        else:
            #if(hensa_flag_az == 0 && hensa_az * pre_hensa_az <= 0)
            #  hensa_flag_az = 1;
            #if(hensa_flag_az == 1)
            ihensa_az += hensa_az
        if math.fabs(hensa_el) >= 0.000:
            ihensa_el = 0
            #hensa_flag_el = 0;
        else:
            #if(hensa_flag_el == 0 && hensa_el * pre_hensa_el <= 0)
                hensa_flag_el = 1;
            #if(hensa_flag_el == 1)
                ihensa_el += hensa_el
        """
        
        """ Original
        self.az_rate = Paz*az_err + Iaz*az_err_integral + Daz*azv_err_avg +azv*1.57
        self.el_rate = Pel*el_err + Iel*el_err_integral + Del*elv_err_avg +elv*1.57
        """
        
        #self.az_rate = target_speed_az * 20.9 + (current_speed_az*20.9 - self.az_rate) * s_az_coeff + p_az_coeff*hensa_az + i_az_coeff*ihensa_az*(self.t1-self.t2) + d_az_coeff*dhensa_az/(self.t1-self.t2)
        #self.el_rate = target_speed_el * 20.9 + p_el_coeff*hensa_el + i_el_coeff*ihensa_el*(self.t1-self.t2) + d_el_coeff*dhensa_el/(self.t1-self.t2)
        self.az_rate = target_speed_az + (self.current_speed_az - self.az_rate) * self.s_az_coeff + self.p_az_coeff*hensa_az + self.i_az_coeff*self.ihensa_az*(self.t1-self.t2) + self.d_az_coeff*dhensa_az/(self.t1-self.t2)
        self.el_rate = target_speed_el + (self.current_speed_el - self.el_rate) * self.s_el_coeff + self.p_el_coeff*hensa_el + self.i_el_coeff*self.ihensa_el*(self.t1-self.t2) + self.d_el_coeff*dhensa_el/(self.t1-self.t2)
        
        self.az_targetspeedmoni = target_speed_az
        self.el_targetspeedmoni = target_speed_el
        self.az_ihensamoni = self.ihensa_az*(self.t1-self.t2)
        self.el_ihensamoni = self.ihensa_el*(self.t1-self.t2)
        
        #for drive.py
        self.t1_moni = self.t1
        self.t2_moni = self.t2
        self.az_pidihensamoni = self.ihensa_az
        self.el_pidihensamoni = self.ihensa_el
        
        
        if math.fabs(az_err) < 8000 and self.az_rate > 10000:
            self.az_rate = 10000
        
        if math.fabs(az_err) < 8000 and self.az_rate < -10000:
            self.az_rate = -10000
        
        if math.fabs(el_err) < 9000 and self.el_rate > 10000:
            self.el_rate = 10000
        
        if math.fabs(el_err) < 7000 and self.el_rate < -8000:
            self.el_rate = -8000
        
        
        #update
        self.az_enc_before = az_enc
        self.el_enc_before = el_enc
        self.az_err_before = az_err
        self.el_err_before = el_err
        
        self.pre_hensa_az = hensa_az
        self.pre_hensa_el = hensa_el
        
        self.pre_az_arcsec = az_arcsec
        self.pre_el_arcsec = el_arcsec
        
        #for negative value of az|el_max_rate
        az_max_rate = math.fabs(az_max_rate)
        el_max_rate = math.fabs(el_max_rate)
        
        if az_max_rate > 16000:
            az_max_rate = 16000
        if el_max_rate > 12000:
            el_max_rate = 12000
        
        #if(az_enc<5*DEG2ARCSEC) rate=min(1000, rate);
        
        softlimit_az_plus = 265.0
        softlimit_az_minus = -265.0
        softlimit_el_plus = 90.1
        softlimit_el_minus = -0.1
        
        
        #limit of dangerous zone
        if (el_enc < softlimit_el_minus * DEG2ARCSEC and self.el_rate < 0 ) or (el_enc > softlimit_el_plus*DEG2ARCSEC and self.el_rate > 0):
            el_max_rate = min(0, el_max_rate)
        if (az_enc < softlimit_az_minus*DEG2ARCSEC and self.az_rate < 0) or (az_enc > softlimit_az_plus*DEG2ARCSEC and self.az_rate > 0):
            az_max_rate = min(1600, az_max_rate); #bug?
        
        #lmit of speed
        if self.az_rate > az_max_rate:
            self.az_rate = az_max_rate
        if self.az_rate < -az_max_rate:
            self.az_rate = -az_max_rate
        if self.el_rate > el_max_rate:
            self.el_rate = el_max_rate
        if self.el_rate < -el_max_rate:
            self.el_rate = -el_max_rate
        
        # arienai ryouiki deno gyakuunndou kinnsi //bug?
        #if az_enc <= -90*DEG2ARCSEC and self.az_rate < 0:
            #self.az_rate = 0
        #if az_enc >= 380*DEG2ARCSEC and self.az_rate > 0:
            #self.az_rate = 0
        
        if az_enc <= softlimit_az_minus*DEG2ARCSEC and self.az_rate < 0:
            self.az_rate = 0
        if az_enc >= softlimit_az_plus*DEG2ARCSEC and self.az_rate > 0:
            self.az_rate = 0
        
        if el_enc <= softlimit_el_minus*DEG2ARCSEC and self.el_rate < 0:
            self.el_rate = 0
        if el_enc >= softlimit_el_plus*DEG2ARCSEC and self.el_rate > 0:
            self.el_rate = 0
        self.t2 = self.t1
        
        az_rate_ref = int(self.az_rate) #??
        el_rate_ref = int(self.el_rate) #??
        return [az_rate_ref, el_rate_ref, m_bAzTrack, m_bElTrack]
    
    def ave_calc(self, az, el):
        array_num = 3
        length = len(self.az_array)
        if length < 3:
            self.az_array.insert(0, az)
            self.el_array.insert(0, el)
        else:
            self.az_array.insert(0, az)
            self.el_array.insert(0, el)
            self.az_array.pop(3)
            self.el_array.pop(3)
        
        ave_az = np.average(self.az_array)
        ave_el = np.average(self.el_array)
        return [ave_az, ave_el]
    
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
        
        median_az = np.median(self.target_az_array)
        median_el = np.median(self.target_el_array)
        return [median_az, median_el]


    
    def stop_move(self, req):
        rospy.loginfo('***subscribe move stop***')
        self.stop_flag = 1
        return
        


    def emergency(self,req):
        if req.data:
            self.emergency_flag = True
            rospy.logwarn('!!!emergency!!!')
            rospy.logwarn('!!!stop azel velocity =>0!!!')
            for i in range(5):
                #self.dio.ctrl.out_word("FBIDIO_OUT1_16", 0)
                #self.dio.output_word([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], 'OUT1_16')
                self.dio.output_word('OUT1_16', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
                #self.dio.ctrl.out_word("FBIDIO_OUT17_32", 0)
                #self.dio.output_word([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], 'OUT17_32')#for aztest
                self.dio.output_word('OUT17_32', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
                time.sleep(0.05)
                self.command_az_speed = 0
                self.command_el_speed = 0
            rospy.logwarn('!!!exit ROS_antenna.py!!!')
            rospy.signal_shutdown('emergency')
            #rospy.on_shutdown(self.emergency_end)
            #rospy.is_shutdown(True)
            sys.exit
            
    def pub_error(self):
        pub = rospy.Publisher('error', Bool, queue_size = 1, latch = True)
        error = Bool()
        error.data = self.error
        pub.publish(error)
        
            
    def pub_status(self):
        rate = rospy.Rate(100)
        pub = rospy.Publisher('status_antenna',Status_antenna_msg, queue_size=1, latch = True)
        pub2 = rospy.Publisher('task_check', Bool, queue_size =1, latch = True)
        while not rospy.is_shutdown():
            #publisher1
            #---------
            status = Status_antenna_msg()
            status.limit_az = self.limit_az
            status.limit_el = self.limit_el
            status.command_az = self.command_az
            status.command_el = self.command_el
            status.emergency = self.emergency_flag
            status.command_azspeed = self.command_az_speed
            status.command_elspeed = self.command_el_speed
            status.node_status = self.node_status
            #publisher2
            #----------
            task = Bool()
            if self.task:
                task.data = True
            else :
                task.data = False
                
            pub.publish(status)
            pub2.publish(task)
            rate.sleep()
            continue

if __name__ == '__main__':
    rospy.init_node('antenna_move')
    ant = antenna_move()
    ant.start_thread()
    print('[ROS_antenna_move.py] : START SUBSCRIBE')
    rospy.Subscriber('list_azel', list_azelmsg, ant.set_parameter)
    rospy.Subscriber('move_stop', String, ant.stop_move)
    rospy.Subscriber('emergency_stop', Bool, ant.emergency)
    rospy.Subscriber('status_encoder', Status_encoder_msg, ant.set_enc_parameter)
    rospy.spin()
    
