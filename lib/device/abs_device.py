#!/usr/bin/env python3

import time
import pyinterface


board_name = 2724
rsw_id = 0

dio = pyinterface.open(board_name, rsw_id)
dio.initialize()


def get_position():
    ret = self.dio.input_byte('IN1_8').to_list()

    if ret[0] == 0 and ret[1] == 1:#ret == 0x02
        position = 'IN'
    elif ret[0] == 1 and ret[1] == 0:#ret == 0x01
        position = 'OUT'
    elif ret[0] == 1 and ret[1] == 1:#ret == 0x03
        position = 'MOVE'
    else:
        print('limit error')
    return position
    
def move(position):
    if position == 'IN':
        pro = [0,0,0,0] #0x00
        buff = [1,0,0,0] #0x01
        print("hot move IN")
    elif position == 'OUT':
        pro = [0,1,0,0] #0x02
        buff = [1,1,0,0] #0x03
        print("hot move OUT")
    else:
        print("Bad command!!")
        return
    dio.output_point(pro, 1)
    time.sleep(1)
    dio.output_point(buff,1)
    return

    
