#!/usr/bin/env python3

import time
import sys
sys.path.append("/home/amigos/ros/src/necst/lib")
import pyinterface
import math

class dome_device(object):
    buffer = [0,0,0,0,0,0]
    stop = [0]
    error = []
    dome_enc = 0
    ###status_parameter
    move_status = ''
    right_act = ''
    right_pos = ''
    left_act = ''
    left_ops = ''
    memb_act = ''
    memb_pos = ''
    remote_status = ''

    def __init__(self):
        board_name = 2724
        rsw_id = 2
        self.dio = pyinterface.open(board_name, rsw_id)
    
    ###Move function
    def move_track(self, enc_az, dome_az):
        dome_az = dome_az/3600.
        enc_az = float(enc_az)
        enc_az = enc_az/3600.
        if math.fabs(enc_az - dome_az) > 1.5 and math.fabs(enc_az - dome_az) < 358.5:#or => and by shiotani
            dir = self.move(enc_az, dome_az*3600, track=True)
            print('tracking', enc_az, dome_az)
        else:
            dir = 1.5
            print("tracking_waiting: ", enc_az, dome_az)
        time.sleep(0.01)
        #print('dome_tracking')
        return dir

    def move(self, dist, pos, track=False):
        pos_arcsec = float(pos)#[arcsec]
        pos = pos_arcsec/3600.
        pos = pos % 360.0
        print("pos: ", pos)
        dist = float(dist) % 360.0
        print("dist: ", dist)
        diff = dist - pos
        dir = diff % 360.0
        print('dir: ', dir)
        """
        if dir < 0:
            dir = dir*(-1)
        """
        if dir == 0:
            return dir
        #"""
        #if dir < 0:
        #    if abs(dir) >= 180:
        #        turn = 'right'
        #    else:
        #        turn = 'left'
        #"""
        else:
            if abs(dir) >= 180:
                turn = 'left'
            else:
                turn = 'right'
        print(abs(dir))
        if abs(dir) < 5.0 or abs(dir) > 355.0:
            speed = 'low'
        elif abs(dir) > 15.0 and abs(dir) < 345.0:
            speed = 'high'
        else:
            speed = 'mid'
        if not abs(dir) < 1.5 or not abs(dir) > 358.5:
            global buffer
            self.buffer[1] = 1
            self.do_output(turn, speed)
            print("track_flag: "track)
            if track:
                time.sleep(0.1)
                return dir
        return dir

    def dome_stop(self):
        buff = [0]
        self.dio.output_point(buff, 2)
        return

    def dome_open(self):
        ret = self.get_door_status()
        if ret[1] != "OPEN" and ret[3] != "OPEN":
            buff = [1, 1]
            self.dio.output_point(buff, 5)
            while ret[1] != 'OPEN' and ret[3] != 'OPEN':
                time.sleep(5)
                ret = self.get_door_status()
        buff = [0, 0]
        self.dio.output_point(buff, 5)
        return

    def dome_close(self):
        ret = self.get_door_status()
        if ret[1] != 'CLOSE' and ret[3] != 'CLOSE':
            buff = [0, 1]
            self.dio.output_point(buff, 5)
            while ret[1] != 'CLOSE' and ret[3] != 'CLOSE':
                time.sleep(5)
                ret = self.get_door_status()
        buff = [0, 0]
        self.dio.output_point(buff, 5)
        return

    def memb_open(self):
        ret = self.get_memb_status()
        if ret[1] != 'OPEN':
            buff = [1, 1]
            self.dio.output_point(buff, 7)
            while ret[1] != 'OPEN':
                time.sleep(5)
                ret = self.get_memb_status()
        buff = [0, 0]
        self.dio.output_point(buff, 7)
        return

    def memb_close(self):
        ret = self.get_memb_status()
        if ret[1] != 'CLOSE':
            buff = [0, 1]
            self.dio.output_point(buff, 7)
            while ret[1] != 'CLOSE':
                time.sleep(5)
                ret = self.get_memb_status()
        buff = [0, 0]
        self.dio.output_point(buff, 7)
        return

    """
    def emergency_stop(self):
        global stop
        self.stop = [1]
        #self.pos.dio.do_output(self.stop, 11, 1)
        self.print_msg('!!EMERGENCY STOP!!')
        return
    """

    def dome_fan(self, fan):
        if fan == 'on':
            fan_bit = [1, 1]
            dio.output_point(fan_bit, 9)
        else:
            fan_bit = [0, 0]
            dio.output_point(fan_bit, 9)
        return

    ###output
    def do_output(self, turn, speed):
        global buffer
        global stop
        if turn == 'right': self.buffer[0] = 0
        else: self.buffer[0] = 1
        if speed == 'low':
            self.buffer[2:4] = [0, 0]
        elif speed == 'mid':
            self.buffer[2:4] = [1, 0]
        else:
            self.buffer[2:4] = [0, 1]
        if self.stop[0] == 1:
            self.buffer[1] = 0
        else: self.buffer[1] = 1
        self.dio.output_point(self.buffer, 1)
        print('do_output')
        return

    ###Status
    def get_action(self):
        ret = self.dio.input_point(1, 1)
        if ret == 0:
            self.move_status = 'OFF'
        else:
            self.move_status = 'DRIVE'
        return
    
    def get_door_status(self):
        ret = self.dio.input_point(2, 6)
        if ret[0] == 0:
            self.right_act = 'OFF'
        else:
            self.right_act = 'DRIVE'
        
        if ret[1] == 0:
            if ret[2] == 0:
                self.right_pos = 'MOVE'
            else:
                self.right_pos = 'CLOSE'
        else:
            self.right_pos = 'OPEN'
        
        if ret[3] == 0:
            self.left_act = 'OFF'
        else:
            self.left_act = 'DRIVE'
        
        if ret[4] == 0:
            if ret[5] == 0:
                self.left_pos = 'MOVE'
            else:
                self.left_pos = 'CLOSE'
        else:
            self.left_pos = 'OPEN'
        return [self.right_act, self.right_pos, self.left_act, self.left_pos]

    def get_memb_status(self):
        ret = self.dio.input_point(8, 3)
        if ret[0] == 0:
            self.memb_act = 'OFF'
        else:
            self.memb_act = 'DRIVE'
        
        if ret[1] == 0:
            if ret[2] == 0:
                self.memb_pos = 'MOVE'
            else:
                self.memb_pos = 'CLOSE'
        else:
            self.memb_pos = 'OPEN'
        return [self.memb_act, self.memb_pos]
    
    def get_remote_status(self):
        ret = self.dio.input_point(11, 1)
        if ret[0] == 0:
            self.remote_status = 'REMOTE'
        else:
            self.remote_status = 'LOCAL'
        return self.remote_status

    def _limit_check(self):
        limit = self.dio.input_point(12, 4)
        ret = 0
        if limit[0:4] == [0,0,0,0]:
            ret = 0
        elif limit[0:4] == [1,0,0,0]:
            ret = 1
        elif limit[0:4] == [0,1,0,0]:
            ret = 2
        elif limit[0:4] == [1,1,0,0]:
            ret = 3
        elif limit[0:4] == [0,0,1,0]:
            ret = 4
        elif limit[0:4] == [1,0,1,0]:
            ret = 5
        elif limit[0:4] == [0,1,1,0]:
            ret = 6
        elif limit[0:4] == [1,1,1,0]:
            ret = 7
        elif limit[0:4] == [0,0,0,1]:
            ret = 8
        elif limit[0:4] == [1,0,0,1]:
            ret = 9
        elif limit[0:4] == [0,1,0,1]:
            ret = 10
        elif limit[0:4] == [1,1,0,1]:
            ret = 11
        elif limit[0:4] == [0,0,1,1]:
            ret = 12
        return ret

    def error_check(self):
        ret = self.dio.input_point(16, 6)
        if ret[0] == 1:
            self.print_error('controll board sequencer error')
        if ret[1] == 1:
            self.print_error('controll board inverter error')
        if ret[2] == 1:
            self.print_error('controll board thermal error')
        if ret[3] == 1:
            self.print_error('controll board communication error')
        if ret[4] == 1:
            self.print_error('controll board sequencer(of dome_door or membrane) error')
        if ret[5] == 1:
            self.print_error('controll board inverter(of dome_door or membrane) error')
        return

    def print_msg(self, msg):
        print(msg)
        return

    def print_error(self, msg):
        self.error.append(msg)
        self.print_error('!!!!ERROR!!!!'+msg)
        return
