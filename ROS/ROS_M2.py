#!/usr/bin/env python

import rospy
import time
import threading
#import M2
#import board_M2
import test_board_M2

from necst.msg import Status_m2_msg
from std_msgs.msg import String



class m2_controller(object):
    #reference  /src/obs/modules/hal/subref.cpp
    #micron is controlled by time
    
    error = []
    m_pos = 0.0
    CW = 0x10
    CCW = 0x11
    LIMIT_CW = 1000
    LIMIT_CCW = -1000
    PULSRATE = 80 #1puls = 0.1micron
    MOTOR_SPEED = 200 # * 10pulses/sec
    m_limit_up = 1
    m_limit_down = 1
    
    
    
    
    def __init__(self):
        pass
    
    def open(self):
        #self.dio = pyinterface.create_gpg2000(ndev)
        #self.board_M2 = board_M2.board()
        self.board_M2 = test_board_M2.board()
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
        
        #bin  = self.dio.ctrl.in_byte("FBIDIO_IN1_8")
        #bin2 = self.dio.ctrl.in_byte("FBIDIO_IN9_16")
        self.board_M2.in_byte("FBIDIO_IN1_8")
        self.board_M2.in_byte("FBIDIO_IN9_16")      

        if bin2 & 0x40:
            self.m_limit_up = 1
        else:
            self.m_limit_up = 0
        if bin2 & 0x80:
            self.m_limit_down = 1
        else:
            self.m_limit_down = 0
        
        for i in range(8):
            if i != 0 and bin == 0:
                buff.append(0)
            if bin % 2 == 0:
                buff.append(0)
            else:
                buff.append(1)
            bin = int(bin/2)
        
        for i in range(8):
            if i != 0 and bin2 == 0:
                buff2.append(0)
            if bin2 % 2 == 0:
                buff2.append(0)
            else:
                buff2.append(1)
            bin2 = int(bin2/2)
        
        #calculate each digit
        total = (buff[0]*1+buff[1]*2+buff[2]*pow(2.,2.)+buff[3]*pow(2.,3.))/100.0
        total = total + (buff[4]*1+buff[5]*2+buff[6]*pow(2.,2.)+buff[7]*pow(2.,3.))/10.0
        total2 = buff2[0]*1+buff2[1]*2+buff2[2]*pow(2.,2.)+buff2[3]*pow(2.,3.)
        total2 = total2 + (buff2[4]*1)*10
        
        self.m_pos = (total+total2)*pow(-1.,(buff2[5]+1))
        
        return self.m_pos
    
    def read_pos(self):
        return self.m_pos
    
    def Strobe(self):
        time.sleep(0.01)
        #self.dio.ctrl.out_byte("FBIDIO_OUT9_16", 0x01)
        self.board_M2.out_byte("FBIDIO_OUT9_16", 0x01)
        time.sleep(0.01)
        #self.dio.ctrl.out_byte("FBIDIO_OUT9_16", 0x00)
	self.board_M2.out_byte("FBIDIO_OUT9_16", 0x00)
        time.sleep(0.01)
        return
    
    def StrobeHOff(self):
        time.sleep(0.01)
        #self.dio.ctrl.out_byte("FBIDIO_OUT9_16", 0x05)
	self.board_M2.out_byte("FBIDIO_OUT9_16", 0x05)
        time.sleep(0.01)
        #self.dio.ctrl.out_byte("FBIDIO_OUT9_16", 0x04)
	self.board_M2.out_byte("FBIDIO_OUT9_16", 0x04)
        time.sleep(0.01)
        return
    
    def move(self, req):
        #move subref
        puls = int(req) * self.PULSRATE
        
        ret = self.get_pos()
        if req/1000.+float(ret) <= -4.0 or req/1000.+float(ret) >= 5.5:
            self.print_error("move limit")
            return
        if self.m_limit_up == 0 and puls < 0:
            self.print_error("can't move up direction")
            return
        if self.m_limit_down == 0 and puls > 0:
            self.print_error("can't move down direction")
            return
        
        self.MoveIndexFF(puls)
        self.get_pos()
        return
    
    
    
    def InitIndexFF(self):
        #initialization?
        
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0x08)
	self.board_M2.out_byte("FBIDIO_OUT1_8", 0x08)
        self.StrobeHOff()
        #step no.
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0xff)
        self.board_M2.out_byte("FBIDIO_OUT1_8", 0xff)
        self.StrobeHOff()
        #vs set
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0x48)
	self.board_M2.out_byte("FBIDIO_OUT1_8", 0x48)
        self.StrobeHOff()
        #5(*10=50)
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0x00)
	self.board_M2.out_byte("FBIDIO_OUT1_8", 0x00)
        self.StrobeHOff()
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0x05)
        self.board_M2.out_byte("FBIDIO_OUT1_8", 0x05)
        self.StrobeHOff()
        #vr set
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0x40)
	self.board_M2.out_byte("FBIDIO_OUT1_8", 0x40)
        self.StrobeHOff()
        
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0x00)
	self.board_M2.out_byte("FBIDIO_OUT1_8", 0x00)
        self.StrobeHOff()
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", self.MOTOR_SPEED)
	self.board_M2.out_byte("FBIDIO_OUT1_8", self.MOTOR_SPEED)
        self.StrobeHOff()
        #su-sd set
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0x50)
	self.board_M2.out_byte("FBIDIO_OUT1_8", 0x50)
        self.StrobeHOff()
        #100(/10=10)
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0x00)
	self.board_M2.out_byte("FBIDIO_OUT1_8", 0x00)
        self.StrobeHOff()
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 100)
	self.board_M2.out_byte("FBIDIO_OUT1_8", 100)
        self.StrobeHOff()
        #position set
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0xc0)
	self.board_M2.out_byte("FBIDIO_OUT1_8", 0xc0)
        self.StrobeHOff()
        #cw
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", self.CW)
	self.board_M2.out_byte("FBIDIO_OUT1_8", self.CW)
        self.StrobeHOff()
        #0
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0x00)
	self.board_M2.out_byte("FBIDIO_OUT1_8", 0x00)
        self.StrobeHOff()
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0x00)
	self.board_M2.out_byte("FBIDIO_OUT1_8", 0x00)
        self.StrobeHOff()
        self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0x00)
	self.board_M2.out_byte("FBIDIO_OUT1_8", 0x00)
        self.StrobeHOff()
        #start
        #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0x18)
	self.board_M2.out_byte("FBIDIO_OUT1_8", 0x18)
        self.StrobeHOff()
        return
    
    def MoveIndexFF(self, puls):
        if puls >= -65535 and puls <= 65535:
            #index mode
            #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0x08)
	    self.board_M2.out_byte("FBIDIO_OUT1_8", 0x08)	
            self.Strobe()
            #step no.
            #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0xff)
	    self.board_M2.out_byte("FBIDIO_OUT1_8", 0xff)
            self.Strobe()
            #position set
            #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0xc0)
	    self.board_M2.out_byte("FBIDIO_OUT1_8", 0xc0)
            self.Strobe()
            #direction
            if puls >= 0:
                #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", self.CW)
	        self.board_M2.out_byte("FBIDIO_OUT1_8", self.CW)
                self.Strobe()
            else:
                #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", self.CCW)
		self.board_M2.out_byte("FBIDIO_OUT1_8", self.CCW)
                self.Strobe()
            #displacement
            #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0x00)
	    self.board_M2.out_byte("FBIDIO_OUT1_8", 0x00)
            self.Strobe()
            #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", (abs(puls) / 256))
	    self.board_M2.out_byte("FBIDIO_OUT1_8", (abs(puls) / 256))
            self.Strobe()
            #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", (abs(puls) % 256))
            self.board_M2.out_byte("FBIDIO_OUT1_8", (abs(puls) % 256))
            self.Strobe()
            #start
            #self.dio.ctrl.out_byte("FBIDIO_OUT1_8", 0x18)
            self.board_M2.out_byte("FBIDIO_OUT1_8", 0x18)
            self.Strobe()
            time.sleep((abs(puls) / self.MOTOR_SPEED / 10.) + 1.)
            self.print_msg("Motor stopped")
        else:
            self.print_msg("Puls number is over.")
            return False
        return True

     def emergency(self,req):
        rospy.loginfo('!!!emergency!!!')
        rospy.loginfo('!!!stop M2!!!')
        self.emergency_flag = 1
        return

    def pub_status(self):
        pub = rospy.Publisher('status_m2',Status_m2_msg, queue_size=10, latch = True)
        msg = Status_m2_msg()

        while not rospy.is_shutdown():
            pos = self.get_pos()
            msg.m2_position = pos
            pub.publish(msg)
            rospy.loginfo(pos)
            time.sleep(0.5)
        return

if __name__ == '__main__':
    m2 = m2_controller()
    m2.open()
    rospy.init_node('m2_controller')
    rospy.loginfo('waiting publish M2')
    m2.start_thread()q2
    sub = rospy.Subscriber('m2', String, m2.move)
    sub = rospy.Subscriber('emergency', String, m2.emergency)
    rospy.spin()






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

