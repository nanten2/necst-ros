#!/usr/bin/env python3

import sys
import time
import struct
import pyinterface

CW = 0x10 # CW = [0,0,0,0,1,0,0,0]
CCW = 0x11 #CCW = [1,0,0,0,1,0,0,0]

PULSRATE = 80 #1puls = 0.1micron
MOTOR_SPEED = 200 # * 10pulses/sec # MOTOR_SPEED_byte = [0,0,0,1,0,0,1,1] # * 10pulses/sec

board_name = 2724
rsw_id = 1



def Strobe():
    time.sleep(0.01)
    dio.output_byte("OUT9_16", [1,0,0,0,0,0,0,0])
    time.sleep(0.01)
    dio.output_byte("OUT9_16", [0,0,0,0,0,0,0,0])
    time.sleep(0.01)
    return

def StrobeHOff():
    time.sleep(0.01)
    dio.output_byte("OUT9_16", [1,0,1,0,0,0,0,0])
    time.sleep(0.01)
    dio.output_byte("OUT9_16", [0,0,1,0,0,0,0,0])
    time.sleep(0.01)
    return

def InitIndexFF():
    #initialization?
    dio.output_byte("OUT1_8", [0,0,0,1,0,0,0,0])
    StrobeHOff()
    #step no.
    dio.output_byte("OUT1_8", [1,1,1,1,1,1,1,1])
    StrobeHOff()
    #vs set
    dio.output_byte("OUT1_8", [0,1,0,0,1,0,0,0])
    StrobeHOff()
    #5(*10=50)
    dio.output_byte("OUT1_8", [0,0,0,0,0,0,0,0])
    StrobeHOff()
    dio.output_byte("OUT1_8", [1,0,1,0,0,0,0,0])
    StrobeHOff()
    #vr set
    dio.output_byte("OUT1_8", [0,0,0,0,0,0,1,0])
    StrobeHOff()
    
    dio.output_byte("OUT1_8", [0,0,0,0,0,0,0,0])
    StrobeHOff()
    dio.output_byte("OUT1_8", MOTOR_SPEED, fmt="<I")
    StrobeHOff()
    #su-sd set
    dio.output_byte("OUT1_8", [0,0,0,0,1,0,1,0])
    StrobeHOff()
    #100(/10=10)
    dio.output_byte("OUT1_8", [0,0,0,0,0,0,0,0])
    StrobeHOff()
    dio.output_byte("OUT1_8", [0,0,1,0,0,1,1,0])
    StrobeHOff()
    #position set
    dio.output_byte("OUT1_8", [0,0,0,0,0,0,1,1])
    StrobeHOff()
    #cw
    dio.output_byte("OUT1_8", CW, fmt="<I")
    StrobeHOff()
    #0
    dio.output_byte("OUT1_8", [0,0,0,0,0,0,0,0])
    StrobeHOff()
    dio.output_byte("OUT1_8", [0,0,0,0,0,0,0,0])
    StrobeHOff()
    dio.output_byte("OUT1_8", [0,0,0,0,0,0,0,0])
    StrobeHOff()
    #start
    dio.output_byte("OUT1_8", [0,0,0,1,1,0,0,0])
    StrobeHOff()
    return

def um_to_puls(dist, status):
    #move subref
    puls = int(dist) * PULSRATE

    if dist/1000.+float(status[0]) <= -4.0 or dist/1000.+float(status[0]) >= 5.5:
        print("move limit")
        return
    if status[1] == 0 and puls < 0:
        print("can't move up direction")
        return
    if status[2] == 0 and puls > 0:
        print("can't move down direction")
        return
    return puls

def MoveIndexFF(puls):
    if puls >= -65535 and puls <= 65535:
        #index mode
        dio.output_byte("OUT1_8", [0,0,0,1,0,0,0,0])
        Strobe()
        #step no.
        dio.output_byte("OUT1_8", [1,1,1,1,1,1,1,1])
        Strobe()
        #position set
        dio.output_byte("OUT1_8", [0,0,0,0,0,0,1,1])
        Strobe()
        #direction
        if puls >= 0:
            dio.output_byte("OUT1_8", CW, fmt="<I")
            Strobe()
        else:
            dio.output_byte("OUT1_8", CCW, fmt="<I")
            Strobe()
        #displacement
        dio.output_byte("OUT1_8", [0,0,0,0,0,0,0,0])
        Strobe()
        dio.output_byte("OUT1_8", int(abs(puls) / 256), fmt="<I")
        Strobe()
        dio.output_byte("OUT1_8", int(abs(puls) % 256), fmt="<I")
        Strobe()
        #start
        dio.output_byte("OUT1_8", [0,0,0,1,1,0,0,0])
        Strobe()
        time.sleep((abs(puls) / MOTOR_SPEED / 10.) + 1.)
        print("Motor stopped")
    else:
        print("Puls number is over.")
        print("Please command x : 10*x [um]")
        return False
    return True


def get_pos():
    buff = []
    buff2 = []
    in1_8  = dio.input_byte("IN1_8").to_list()
    in9_16 = dio.input_byte("IN9_16").to_list()
        
    if in9_16[6] == 1: # [0,0,0,0,0,0,1,0]:
        m_limit_up = 1
    else:
        m_limit_up = 0
    if in9_16[7] == 1: # [0,0,0,0,0,0,0,1]:
        m_limit_down = 1
    else:
        m_limit_down = 0
        
            
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
        
    m_pos = (total+total2)*pow(-1.,(buff2[5]+1))
    return [m_pos, m_limit_up, m_limit_down]

    
#setup
dio = pyinterface.open(board_name, rsw_id)
dio.initialize()
InitIndexFF()
