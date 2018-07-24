#!/usr/bin/env python3
"""
history
2018/06/08 edited by Shiotani
ROS_antenna_move.py & S_ROS_antenna_move.py
"""
import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import rospy
import numpy
import time
import threading
import math
sys.path.append("/home/necst/ros/src/necst/lib")
import antenna_device
import numpy as np
import struct
import pyinterface

#ROS/import field
#----------------
from necst.msg import Status_antenna_msg
from necst.msg import List_coord_msg
from necst.msg import Status_encoder_msg
from necst.msg import Bool_necst

node_name = 'antenna_move'

class antenna_move(object):
    
    
    #initial parameter
    #-----------------
    parameters = {
        'az_list':[],#[0]*300,
        'el_list':[],#[0]*300,
        'start_time_list':[],#0,
        'flag':0
        }

    enc_parameter = {
        'az_enc':0,
        'el_enc':0
        }
    B_time = 1

    stop_flag = True #False
    task = 0
    error = False #False = ok
    emergency_flag = False
    limit_flag = 0###(0/1=okay/NG)
    limit_az = True###(True/False = okay/limit)
    limit_el = True
    command_az = 0
    command_el = 0

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
    
    #error_box = [0]*32###0921
    #azv_before = elv_before = 0
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
    
    #fordrive.py
    az_pidihensamoni = 0
    el_pidihensamoni = 0
    
    
    
    th_az = 0
    th_el = 0
    server_flag = []
    time_list = [0,0,0,0,0]
    save_time = 0
    end_flag = 0

    
    def __init__(self):
        self.dev = antenna_device.antenna_device()
        self.start_time = time.time()
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
        if not req.time_list:
            return
        else:
            pass
        if not self.stop_flag and self.start_time<req.time_list[0]:
            print("st,ct", self.stop_flag, self.start_time, req.time_list[0])
            if self.parameters['start_time_list'] != []:
                time_len = len(self.parameters['start_time_list'])
                for i in range(time_len):
                    if req.time_list[0]< self.parameters['start_time_list'][-1]:
                        del self.parameters['az_list'][-1]
                        del self.parameters['el_list'][-1]
                        del self.parameters['start_time_list'][-1]
                    else:
                        break
            else:
                pass
            self.parameters['az_list'].extend(req.x_list)
            self.parameters['el_list'].extend(req.y_list)
            self.parameters['start_time_list'].extend(req.time_list)
        else:
            self.parameters['az_list'] = []
            self.parameters['el_list'] = []
            self.parameters['start_time_list'] = []
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
        loop = 0
        first_st = self.parameters['start_time_list']
        for i in range(20):
            n = len(self.parameters['az_list'])
            st = self.parameters['start_time_list']
            if st == []:
                return
            ct = time.time()

            if loop == 19:
                pass
            elif ct - st[n-1] >=0 and first_st == st:
                loop += 1
                time.sleep(0.1)
                continue
            else:
                break
            rospy.loginfo('!!!azel_list is end!!!')
            self.stop_flag = 1
            for i in range(5):
                #self.dio.output_word('OUT1_16', [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#az
                #self.dio.output_word('OUT17_32',[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])#el
                self.command_az_speed = 0
                self.command_el_speed = 0
                time.sleep(0.25)
            return

        if self.parameters['az_list'] == []:
            return
        else:
            for i in range(len(self.parameters['az_list'])):
                st2 = st[i+1]
                num = i+1
                if st2 - ct >0:
                    break
            if num == len((self.parameters['az_list'])):
                return
            param = self.parameters
            x1 = param['az_list'][num-1]
            x2 = param['az_list'][num]
            y1 = param['el_list'][num-1]
            y2 = param['el_list'][num]
            return (x1,x2,y1,y2,st2)
        
    def act_azel(self):
        while True:
            if self.stop_flag:
                time.sleep(1)
                self.command_az_speed = 0
                self.command_el_speed = 0
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
                tar_az = ret[0] + az*(c-st)*10
                tar_el = ret[2] + el*(c-st)*10
                if tar_az > 240*3600. or tar_el < -240*3600.:
                    self.stop_flag = True#False?
                    print('!!!target az limit!!! : ', tar_az)
                    continue
                if tar_el > 89*3600. or tar_el < 0:
                    self.stop_flag = True#False?
                    print('!!!target el limit!!! : ', tar_el)
                    continue
                self.command_az = tar_az
                self.command_el = tar_el
                d_t = st - c
                a_time3=time.time()
                if self.emergency_flag:
                    time.sleep(0.1)
                    continue
                self.azel_move(tar_az,tar_el,10000,12000)
                time.sleep(0.01)           


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
    """
    #ROS_version
    #cur_az cur_el means encoder_azel
    def azel_move(self, az_arcsec, el_arcsec, az_max_rate, el_max_rate):
        test_flag = 1

        self.indaz = az_arcsec
        self.indel = el_arcsec
        
        self.enc_az = self.enc_parameter['az_enc']
        self.enc_el = self.enc_parameter['el_enc']
        if True:#shiotani changed   
        #if abs(az_arcsec - self.enc_az) >= 1 or abs(el_arcsec - self.enc_el) > 1:###self.enc_az is provisonal
            b_time = time.time()
            
            ret = self.dev.move_azel(az_arcsec, el_arcsec, az_max_rate, el_max_rate, self.enc_az, self.enc_el)
            rospy.loginfo(ret)
            if stop_flag:
                rospy.logwarn('')
                sys.exit()
            
            interval = time.time()-b_time
            if interval <= 0.01:
                time.sleep(0.01-interval)
                pass
                    
            return 1
    
    def stop_move(self, req):
        if time.time() - self.start_time < 1:
            return
        
        if req.data == False:
            pass
        else:
            rospy.loginfo('***subscribe move stop***')
            self.parameters['az_list'] = []
            self.parameters['el_list'] = []
            self.parameters["start_time_list"] = []
        
        self.stop_flag = req.data
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
        pub = rospy.Publisher('error', Bool_necst, queue_size = 1, latch = True)
        error = Bool_necst()
        error.data = self.error
        error.from_node = node_name
        error.timestamp = time.time()
        pub.publish(error)
        
    def pub_status(self):
        pub = rospy.Publisher('status_antenna',Status_antenna_msg, queue_size=1, latch = True)
        pub2 = rospy.Publisher('task_check', Bool_necst, queue_size =1, latch = True)
        status = Status_antenna_msg()
        task = Bool_necst()
        while not rospy.is_shutdown():
            #publisher1
            #---------
            status.limit_az = self.limit_az
            status.limit_el = self.limit_el
            status.command_az = self.command_az
            status.command_el = self.command_el
            status.emergency = self.emergency_flag
            status.command_azspeed = self.command_az_speed
            status.command_elspeed = self.command_el_speed
            status.node_status = self.node_status
            status.from_node = node_name
            status.timestamp = time.time()
            #publisher2
            #----------
            if self.task:
                task.data = True
            else :
                task.data = False
                pass
            task.from_node = node_name
            task.timestamp = time.time()
            pub.publish(status)
            pub2.publish(task)
            time.sleep(0.001)
            continue

if __name__ == '__main__':
    rospy.init_node(node_name)
    ant = antenna_move()
    ant.start_thread()
    print('[ROS_antenna_move.py] : START SUBSCRIBE')
    rospy.Subscriber('list_azel', List_coord_msg, ant.set_parameter, queue_size=1)
    rospy.Subscriber('move_stop', Bool_necst, ant.stop_move)
    rospy.Subscriber('emergency_stop', Bool_necst, ant.emergency)
    rospy.Subscriber('status_encoder', Status_encoder_msg, ant.set_enc_parameter, queue_size=1)
    rospy.spin()    
