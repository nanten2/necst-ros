#! /usr/bin/env python3

import time
import pyinterface

board_name = 2724
output_rsw_id = 1
input_rsw_id = 0

dio = pyinterface.open(board_name, output_rsw_id)
#dio_input = pyinterface.open(self.board_name, self.input_rsw_id)

""" This publish is ROS_limit_check.py 
def get_position():
    pos = self.dio_input.input_byte("IN1_8").to_list()
    if pos[0] == 1 and pos[1] == 1:
        self.contactor_pos = "on"
    else:
        self.contactor_pos = "off"
        pass
    if pos[2] == 1 and pos[3] == 1:
        self.drive_pos = "on"
    else:
        self.drive_pos = "off"
        pass
    print("Current position is : ")
    print("drive : ",self.drive_pos, "contactor : ", self.contactor_pos)
    return [self.drive_pos, self.contactor_pos]
"""

def move_drive(position):
    
    if position == "on":
        dio.output_point([1,1], 1) #output_byte([1,1,0,0,0,0,0,0],OUT1_8)
    elif position == "off":
        dio.output_point([0,0], 1) #output_byte([0,0,0,0,0,0,0,0], 'OUT1_8')
    else:
        pass
    return

def move_contactor(position):
    
    if position == "on":
        dio.output_point([1,1,1,1], 9) #output_byte([1,1,1,1,0,0,0,0], 'OUT9_16')
    elif position == "off":
        dio.output_point([0,0,0,0], 9) #output_byte([0,0,0,0,0,0,0,0], 'OUT9_16')
    else:
        pass
    return
