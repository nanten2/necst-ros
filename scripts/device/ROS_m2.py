#!/usr/bin/env python3

"""
create : okuda
correction : kondo 2017/12/19
"""

import sys
import rospy
import time
import threading
import struct
import pyinterface

#from necst.msg import Status_m2_msg
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int64


class m2_controller(object):
    #reference  /src/obs/modules/hal/subref.cpp
    #micron is controlled by time
    
    error = []
    m_pos = 0.0
    CW = 0x10 # CW = [0,0,0,0,1,0,0,0]
    CCW = 0x11 #CCW = [1,0,0,0,1,0,0,0]
    LIMIT_CW = 1000
    LIMIT_CCW = -1000
    PULSRATE = 80 #1puls = 0.1micron
    MOTOR_SPEED = 200 # * 10pulses/sec # MOTOR_SPEED_byte = [0,0,0,1,0,0,1,1] # * 10pulses/sec
    m_limit_up = 1
    m_limit_down = 1
    puls = ""
    past_puls = ""
    
    
    def __init__(self):
        self.stop_thread = threading.Event()
        self.thread = threading.Thread(target=self.move_thread)
        self.thread.setDaemon(True)
        self.thread.start()
        pass
    
    def open(self,ndev):
        #self.dio = pyinterface.create_gpg2000(ndev)
        self.dio = pyinterface.open(2724,1)
        #self.board_M2 = board_M2.board()
        #self.board_M2 = test_board_M2.board()
        self.InitIndexFF()
        self.get_pos()
        pass

    def start_thread(self):
        th = threading.Thread(target = self.pub_status)
        th.setDaemon(True)
        th.start()
        return
    
    def print_msg(self, msg):
        print(msg)
        return
    
    def print_error(self, msg):
        self.error.append(msg)
        self.print_msg('!!!! ERROR !!!! ' + msg)
        return
    
    def get_pos(self):
        buff = []
        buff2 = []
        in1_8  = self.dio.input_byte("IN1_8").to_list()
        in9_16 = self.dio.input_byte("IN9_16").to_list()
        #bin = self.board_M2.in_byte("FBIDIO_IN1_8")
        #bin2 = self.board_M2.in_byte("FBIDIO_IN9_16")      

        if in9_16 == [0,0,0,0,0,0,1,0]:
            self.m_limit_up = 1
        else:
            self.m_limit_up = 0
        if in9_16 == [0,0,0,0,0,0,0,1]:
            self.m_limit_down = 1
        else:
            self.m_limit_down = 0


        for i in range(8):
            if i != 0 and in1_8 == [0,0,0,0,0,0,0,0]:
                buff.append(0)
            if in1_8[i] == 0:
                buff.append(0)
            else:
                buff.append(1)
        
        for i in range(8):
            if i != 0 and in9_16 == [0,0,0,0,0,0,0,0]:
                buff2.append(0)
            if in9_16[i]== 0:
                buff2.append(0)
            else:
                buff2.append(1)
        
        #calculate each digit
        total = (buff[0]*1+buff[1]*2+buff[2]*pow(2.,2.)+buff[3]*pow(2.,3.))/100.0
        total = total + (buff[4]*1+buff[5]*2+buff[6]*pow(2.,2.)+buff[7]*pow(2.,3.))/10.0
        total2 = buff2[0]*1+buff2[1]*2+buff2[2]*pow(2.,2.)+buff2[3]*pow(2.,3.)
        total2 = total2 + (buff2[4]*1)*10
        
        self.m_pos = (total+total2)*pow(-1.,(buff2[5]+1))
        #rospy.logwarn(self.m_pos)
        return self.m_pos
    
    def read_pos(self):
        return self.m_pos
    
    def Strobe(self):
        time.sleep(0.01)
        self.dio.output_byte("OUT9_16", [1,0,0,0,0,0,0,0])
        time.sleep(0.01)
        self.dio.output_byte("OUT9_16", [0,0,0,0,0,0,0,0])
        time.sleep(0.01)
        return
    
    def StrobeHOff(self):
        time.sleep(0.01)
        self.dio.output_byte("OUT9_16", [1,0,1,0,0,0,0,0])
        time.sleep(0.01)
        self.dio.output_byte("OUT9_16", [0,0,1,0,0,0,0,0])
        time.sleep(0.01)
        return
    
    
    def move(self, req):
        #move subref
        puls = int(req.data) * self.PULSRATE
        rospy.logerr(req.data)
        ret = self.get_pos()
        print("ret",ret)
        if req.data/1000.+float(ret) <= -4.0 or req.data/1000.+float(ret) >= 5.5:
            self.print_error("move limit")
            #return
        print(self.m_limit_down)
        if self.m_limit_up == 0 and puls < 0:
            self.print_error("can't move up direction")
            return
        if self.m_limit_down == 0 and puls > 0:
            self.print_error("can't move down direction")
            return
        self.puls = puls
        
        return

    def move_thread(self):
        while not rospy.is_shutdown():
            if self.puls == "":
                pass
            elif self.puls != self.past_puls:
                print("move start")
                self.MoveIndexFF(self.puls)
                self.get_pos()
                self.past_puls = self.puls
                print("move end")
            else:
                pass
            
    
    
    
    def InitIndexFF(self):
        #initialization?
        self.dio.output_byte("OUT1_8", [0,0,0,1,0,0,0,0])
        self.StrobeHOff()
        #step no.
        self.dio.output_byte("OUT1_8", [1,1,1,1,1,1,1,1])
        self.StrobeHOff()
        #vs set
        self.dio.output_byte("OUT1_8", [0,1,0,0,1,0,0,0])
        self.StrobeHOff()
        #5(*10=50)
        self.dio.output_byte("OUT1_8", [0,0,0,0,0,0,0,0])
        self.StrobeHOff()
        self.dio.output_byte("OUT1_8", [1,0,1,0,0,0,0,0])
        self.StrobeHOff()
        #vr set
        self.dio.output_byte("OUT1_8", [0,0,0,0,0,0,1,0])
        self.StrobeHOff()
        
        self.dio.output_byte("OUT1_8", [0,0,0,0,0,0,0,0])
        self.StrobeHOff()
        self.dio.output_byte("OUT1_8", self.MOTOR_SPEED, fmt="<I")
        self.StrobeHOff()
        #su-sd set
        self.dio.output_byte("OUT1_8", [0,0,0,0,1,0,1,0])
        self.StrobeHOff()
        #100(/10=10)
        self.dio.output_byte("OUT1_8", [0,0,0,0,0,0,0,0])
        self.StrobeHOff()
        self.dio.output_byte("OUT1_8", [0,0,1,0,0,1,1,0])
        self.StrobeHOff()
        #position set
        self.dio.output_byte("OUT1_8", [0,0,0,0,0,0,1,1])
        self.StrobeHOff()
        #cw
        self.dio.output_byte("OUT1_8", self.CW, fmt="<I")
        self.StrobeHOff()
        #0
        self.dio.output_byte("OUT1_8", [0,0,0,0,0,0,0,0])
        self.StrobeHOff()
        self.dio.output_byte("OUT1_8", [0,0,0,0,0,0,0,0])
        self.StrobeHOff()
        self.dio.output_byte("OUT1_8", [0,0,0,0,0,0,0,0])
        self.StrobeHOff()
        #start
        self.dio.output_byte("OUT1_8", [0,0,0,1,1,0,0,0])
        self.StrobeHOff()
        return
    
    def MoveIndexFF(self, puls):
        if puls >= -65535 and puls <= 65535:
            #index mode
            self.dio.output_byte("OUT1_8", [0,0,0,1,0,0,0,0])
            self.Strobe()
            #step no.
            self.dio.output_byte("OUT1_8", [1,1,1,1,1,1,1,1])
            self.Strobe()
            #position set
            self.dio.output_byte("OUT1_8", [0,0,0,0,0,0,1,1])
            self.Strobe()
            #direction
            if puls >= 0:
                self.dio.output_byte("OUT1_8", self.CW, fmt="<I")
                self.Strobe()
            else:
                self.dio.output_byte("OUT1_8", self.CCW, fmt="<I")
                self.Strobe()
            #displacement
            self.dio.output_byte("OUT1_8", [0,0,0,0,0,0,0,0])
            self.Strobe()
            self.dio.output_byte("OUT1_8", int(abs(puls) / 256), fmt="<I")
            self.Strobe()
            self.dio.output_byte("OUT1_8", int(abs(puls) % 256), fmt="<I")
            self.Strobe()
            #start
            self.dio.output_byte("OUT1_8", [0,0,0,1,1,0,0,0])
            self.Strobe()
            time.sleep((abs(puls) / self.MOTOR_SPEED / 10.) + 1.)
            self.print_msg("Motor stopped")
        else:
            self.print_msg("Puls number is over.")
            self.print_msg("Please command x : 10*x [um]")
            return False
        return True

    def emergency(self,req):
        rospy.loginfo('!!!emergency!!!')
        rospy.loginfo('!!!stop M2!!!')
        self.emergency_flag = 1
        return

    def pub_status(self):
        pub = rospy.Publisher('status_m2',Float64, queue_size=10, latch = True)
        msg = Float64()
        
        while not rospy.is_shutdown():
            pos = self.get_pos()
            msg.data = pos
            pub.publish(msg)
            rospy.loginfo(msg)
            time.sleep(0.5)
        return

if __name__ == '__main__':
    m2 = m2_controller()
    m2.open(2)
    rospy.init_node('m2_controller')
    rospy.loginfo('waiting publish M2')
    m2.start_thread()
    sub = rospy.Subscriber('m2', Int64, m2.move)
    sub = rospy.Subscriber('emergency', String, m2.emergency)
    rospy.spin()


"""



def m2_client(host, port):
    client = pyinterface.server_client_wrapper.control_client_wrapper(m2_controller, host, port)
    return client

def m2_monitor_client(host, port):
    client = pyinterface.server_client_wrapper.monitor_client_wrapper(m2_controller, host, port)
    return client

def start_m2_server(port1 = 9999, port2 = 9998):
    m2 = m2_controller()
    server = pyinterface.server_client_wrapper.server_wrapper(m2,'', port1, port2)
    server.start()
    return server

"""
