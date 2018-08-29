#!/usr/bin/env python3

import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import rospy
import numpy
import time
import threading
sys.path.append("/home/amigos/ros/src/necst/lib/device")

#ROS/import field
#----------------
from necst.msg import Status_antenna_msg
from necst.msg import List_coord_msg
from necst.msg import Status_encoder_msg
from necst.msg import Bool_necst

node_name = 'antenna_move'
mode = sys.argv[1]#'Actual/Simulator'
if mode == 'Actual':
    import antenna_device
elif mode == 'Simulator':
    import S_antenna_device as antenna_device
else:
    rospy.logwarn('Launch this node with launch file')

class antenna_move(object):
    
    
    #initial parameter
    #-----------------
    parameters = {
        'az_list':[],
        'el_list':[],
        'start_time_list':[],
        }

    enc_parameter = {
        'az_enc':0,
        'el_enc':0
        }

    stop_flag = True #False
    error = False #False = ok
    emergency_flag = False
    limit_az = True###(True/False = okay/limit)
    limit_el = True
    command_az = 0
    command_el = 0


    tracking_status = False
    list_coord = ''
    
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

    def set_tracking(self, req):
        self.tracking_status = req.data
        
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
        self.list_coord = req.coord
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

    def limit_check(self, az, el):
        """
        DESCRIPTION
        ===========
        This function checks limit azel
        """
        if az > 240*3600. or az < -240*3600.: #240?
            self.stop_flag = True
            print('!!!target az limit!!! : ', az)
            rospy.logwarn('!!!limit az!!!')
            self.error = True
            return True

        if el > 89*3600. or el < 0:
            self.stop_flag = True
            print('!!!target el limit!!! : ', el)
            rospy.logwarn('!!!limit el!!!')
            self.error = True
            return True
        
        else:
            return False
    
    def time_check(self, st):
        ct = time.time()

        if ct - st[-1] >= 0:
            rospy.loginfo('!!!azel_list is end!!!')
            self.stop_flag = True
            self.dev.command_az_speed = 0
            self.dev.command_el_speed = 0
            time.sleep(0.25)
            return 
        return ct

    def comp(self):
        """
        DESCRIPTION
        ===========
        This function determine target Az and El from azel_list
        """
        st = self.parameters['start_time_list']
        if st == []:
            return

        ct = self.time_check(st)
        if ct == None or self.parameters['az_list'] == []:
            return
        else:
            num = numpy.where(numpy.array(st) > ct)[0][0]
            st2 = st[num]
            param = self.parameters
            if len(param['az_list']) > num:
                az_1 = param['az_list'][num-1]
                az_2 = param['az_list'][num]
                el_1 = param['el_list'][num-1]
                el_2 = param['el_list'][num]
            else:
                return
            return (az_1,az_2,el_1,el_2,st2)
        
    def act_azel(self):
        ###for azel_move check
        command_az_before = ''
        command_el_before = ''
        ###
        while True:
            if self.stop_flag:
                time.sleep(1)
                self.dev.command_az_speed = 0
                self.dev.command_el_speed = 0
                continue
            
            if self.emergency_flag:
                time.sleep(0.1)
                continue
            
            if self.parameters['az_list'][-1] == command_az_before and self.parameters['el_list'][-1] == command_el_before and self.list_coord == 'altaz':###for azel_move check
                self.stop_flag = True
                print('list_check')
                continue

            print(self.list_coord)
            if self.list_coord == 'altaz':#azel_move
                print('altaz_check')
                if self.tracking_status == False:
                    pass
                else:
                    command_az_before = self.command_az
                    command_el_before = self.command_el
                    self.stop_flag = True
                    print('altaz_check#stop_flag True')
            else:
                pass

            ret = self.comp()
            if ret == None:
                time.sleep(0.1)
                continue
            else:
                hensa_az = ret[1] - ret[0]
                hensa_el = ret[3] - ret[2]
                current_time = time.time()
                start_time = ret[4]
                tar_az = ret[0] + hensa_az*(current_time-start_time)*10
                tar_el = ret[2] + hensa_el*(current_time-start_time)*10
                
                if self.limit_check(tar_az, tar_el):
                    time.sleep(0.1)
                    continue

                self.command_az = tar_az
                self.command_el = tar_el

                b_time = time.time()

                ret = self.dev.move_azel(tar_az, tar_el, self.enc_parameter['az_enc'], self.enc_parameter['el_enc'])
                rospy.loginfo(ret)
                
                interval = time.time() - b_time
                if interval <= 0.01:
                    time.sleep(0.01 - interval)
                    pass
        return

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
            self.dev.emergency_stop()
            rospy.logwarn('!!!exit ROS_antenna.py!!!')
            rospy.signal_shutdown('emergency')
            sys.exit()
            
    def pub_error(self):
        pub = rospy.Publisher('error', Bool_necst, queue_size = 1, latch = True)
        error = Bool_necst()
        error.data = self.error
        error.from_node = node_name
        error.timestamp = time.time()
        pub.publish(error)
        
    def pub_status(self):
        pub = rospy.Publisher('status_antenna',Status_antenna_msg, queue_size=1, latch = True)
        status = Status_antenna_msg()
        while not rospy.is_shutdown():
            #publisher
            #---------
            status.limit_az = self.limit_az
            status.limit_el = self.limit_el
            status.command_az = self.command_az
            status.command_el = self.command_el
            status.emergency = self.emergency_flag
            status.command_azspeed = self.dev.command_az_speed
            status.command_elspeed = self.dev.command_el_speed
            status.node_status = self.node_status
            status.from_node = node_name
            status.timestamp = time.time()
            pub.publish(status)
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
    rospy.Subscriber('tracking_check', Bool_necst, ant.set_tracking, queue_size=1)
    rospy.spin()    
