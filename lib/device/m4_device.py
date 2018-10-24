#!/usr/bin/env python3

import time
import pyinterface

board_name = 7204
rsw_id = 0

mtr = pyinterface.open(board_name, rsw_id)
mtr.initialize()
mtr.set_limit_config('LOGIC', '+EL -EL', axis=1)


def get_pos():
    print(mtr.get_counter())
    status = mtr.get_status()
    #print(status)
    """
    if status["busy"] == False and status["limit"]["+EL"] == 1:
        status["limit"]["-EL"] = 0
    """
    if status["busy"] == True:
        position = 'MOVE'
    elif status["limit"]["+EL"] == 0: #status == 0x0004:
        #OUT
        position = 'SMART'
    elif status["limit"]["-EL"] == 0:
        #IN
        position = 'NAGOYA'
    else:
        print('limit error')
        position = 'ERROR'
    return position


def move(position):
    if position == 'SMART' or position == "OUT":
        #nstep = 60500
        step=1
        print('m4 move out')
    elif position == 'NAGOYA' or position == "IN":
        #nstep = -60500
        step=-1
        print('m4 move in')
    else:
        print('parameter error')
        return
    mtr.set_motion(mode="JOG",step=step)
    print("set_motion")
    mtr.start_motion(mode="JOG")
    print("start_motion")
    if mtr.get_status()['busy'] == False:
        print("status busy")
        counter_reset(position)
    else:
        pass
    time.sleep(0.5)
    return

def counter_reset(position):
    is_stop = not mtr.get_status(axis=1)['busy']
    
    if (position=='NAGOYA') & is_stop:
        mtr.set_counter(0, axis=1)
        pass
    return

def stop():
    mtr.stop()
    return
