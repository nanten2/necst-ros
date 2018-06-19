#!/usr/bin/env python3

import time
import math
import threading
import sys
#ros
import rospy
from necst.msg import Status_encoder_msg
from necst.msg import Status_dome_msg
from necst.msg import Dome_msg

node_name = "dome"

class dome_controller(object):

    ###parameter
    enc_az = 0
    dome_enc = 0
    move_status = 'DRIVE'#OFF/DRIVE
    right_act = 'DRIVE'#OFF/DRIVE
    right_pos = 'CLOSE'#OPEN/MOVE/CLOSE
    left_act = 'DRIVE'#OFF/DRIVE
    left_pos = 'CLOSE'#OPEN/MOVE/CLOSE
    memb_act = 'DRIVE'#OFF/DRIVE
    memb_pos = 'CLOSE'#OPEN/MOVE/CLOSE
    remote_status = 'REMOTE'#REMOTE/LOCAL

    ###flag
    flag = 0

    #paramter(pub)
    parameters = {
        'command':0
        }
    paralist = ['start']
    parameter_az = 0
    #
    buffer = [0,0,0,0,0,0]
    
    def __init__(self):
        self.start_status_check()
        pass

    def move_track(self):
        print('dome_trakcking start', self.end_flag)
        ret = self.read_domepos()
        while not self.end_flag:
            enc_az = float(self.enc_az)
            dome_az = self.read_domepos()
            dome_az = dome_az/3600.
            #self.dome_limit()
            enc_az = float(enc_az)
            enc_az = enc_az/3600.
            if math.fabs(enc_az - dome_az) >= 2.0:
                self.move(enc_az, track=True)
            time.sleep(0.01)
            if self.end_flag == True:
                break
            print('dome_tracking')
    
    def test(self, num): #for track_test
        self.start_thread()
        time.sleep(num)
        self.end_thread()
        return
    
    def print_msg(self,msg):
        print(msg)
        return
    
    def print_error(self,msg):
        self.error.append(msg)
        self.print_msg('!!!!ERROR!!!!'+msg)
        return
    
    def move_org(self):
        dist = 90
        self.move(dist)    #move_org
        return
    
    def move(self, dist, track=False):
        while not self.end_flag:
            pos_arcsec = float(self.dome_enc)#[arcsec]
            pos = pos_arcsec/3600.
            pos = pos % 360.0
            dist = float(dist) % 360.0
            diff = dist - pos
            dir = diff % 360.0
            if dir < 0:
                dir = dir*(-1)
            
            if pos == dist: return
            if dir < 0:
                if abs(dir) >= 180:
                    turn = 'right'
                else:
                    turn = 'left'
            else:
                if abs(dir) >= 180:
                    turn = 'left'
                else:
                    turn = 'right'
            if abs(dir) < 5.0 or abs(dir) > 355.0 :
                speed = 'low'
            elif abs(dir) > 15.0 and abs(dir) < 345.0:###or => and
                speed = 'high'
            else:
                speed = 'mid'
            if not abs(dir) < 0.5:
                global buffer
                self.buffer[1] = 1
                self.calc(turn, speed)
                print(track)
                if track:
                    time.sleep(0.1)
                    return
                while dir != 0:
                    pos_arcsec = float(self.dome_enc)
                    pos = pos_arcsec/3600.
                    pos = pos % 360.0
                    dist = dist % 360.0
                    diff = dist - pos
                    dir = diff % 360.0
                    if abs(dir) <= 0.5 or self.end_flag == True:
                        dir = 0
                    else:
                        if dir < 0:
                            if abs(dir) >= 180:
                                turn = 'right'
                            else:
                                turn = 'left'
                        else:
                            if abs(dir) >= 180:
                                turn = 'left'
                            else:
                                turn = 'right'
                                
                        if abs(dir) < 5.0 or abs(dir) > 355.0:
                            speed = 'low'
                        elif abs(dir) > 20.0 and abs(dir) < 340.0:###or => and
                            speed = 'high'
                        else:
                            speed = 'mid'
                        self.calc(turn, speed)
                    time.sleep(0.1)
            
            self.dome_stop()
            break
        return

    def calc(self, turn, speed):
        if speed == 'high':
            dome_speed = 3000
        elif speed == 'mid':
            dome_speed = 1500
        elif speed == 'low':
            dome_speed = 300
        else :
            dome_speed = 0
            
        if turn == 'right':
            self.dome_enc += dome_speed
        else:
            self.dome_enc -= dome_speed
        if self.dome_enc >=360.*3600.:
            self.dome_enc -= 360.*3600.
        elif self.dome_enc <=-360.*3600.:
            self.dome_enc += 360.*3600.
        pass
    def dome_stop(self):
        buff = [0]
        self.flag = 0
        return
    
    def dome_open(self):
        ret = self.get_door_status()
        if ret[1] != 'OPEN' and ret[3] != 'OPEN':
            buff = [1, 1]
            self.right_pos = 'MOVE'
            self.left_pos = 'MOVE'
            time.sleep(3)
            self.right_pos = 'OPEN'
            self.left_pos = 'OPEN'
            d_door = self.get_door_status()
            """
            while ret[1] != 'OPEN':
                time.sleep(0.01)
                ret = self.get_door_status()
            """
        buff = [0, 0]
        return
    
    def dome_close(self):
        ret = self.get_door_status()
        if ret[1] != 'CLOSE' and ret[3] != 'CLOSE':
            buff = [0, 1]
            self.right_pos = 'MOVE'
            self.left_pos = 'MOVE'
            time.sleep(3)
            self.right_pos = 'CLOSE'
            self.left_pos = 'CLOSE'
            """
            while ret[1] != 'CLOSE':
                time.sleep(0.01)
                ret = self.get_door_status()
            """
        buff = [0, 0]
        #self.dio.do_output(buff, 5, 2)
        #self.dio.output_point(buff, 5)
        return
    
    def memb_open(self):
        ret = self.get_memb_status()
        if ret[1] != 'OPEN':
            buff = [1, 1]
            #self.dio.do_output(buff, 7, 2)
            #self.dio.output_point(buff, 7)
            self.memb_pos = 'MOVE'
            time.sleep(3)
            self.memb_pos = 'OPEN'
            """
            while ret[1] != 'OPEN':
                time.sleep(0.01)
                #ret = self.get_memb_status()
            """
        buff = [0, 0]
        #self.dio.do_output(buff, 7, 2)
        #self.dio.output_point(buff, 7)
        return
    
    def memb_close(self):
        ret = self.get_memb_status()
        if ret[1] != 'CLOSE':
            buff = [0, 1]
            #self.dio.do_output(buff, 7, 2)
            #self.dio.output_point(buff, 7)
            self.memb_pos = 'MOVE'
            time.sleep(3)
            self.memb_pos = 'CLOSE'
            """
            while ret[1] != 'CLOSE':
                time.sleep(0.01)
                ret = self.get_memb_status()
            """
        buff = [0, 0]
        #self.dio.do_output(buff, 7, 2)
        #self.dio.output_point(buff, 7)
        return
    
    """
    def emergency_stop(self):
        global stop
        dome_controller.stop = [1]
        #self.pos.dio.do_output(self.stop, 11, 1)
        self.print_msg('!!EMERGENCY STOP!!')
        return
    """
    
    def dome_fan(self, fan):
        if fan == 'on':
            fan_bit = [1, 1]
            #self.dio.do_output(fan_bit, 9, 2)
            #self.dio.output_point(fan_bit, 9)
        else:
            fan_bit = [0, 0]
            #self.dio.do_output(fan_bit, 9, 2)
            #self.dio.output_point(fanbit, 9)
        return
    
    def get_count(self):
        self.count = self.dome_pos.dome_encoder_acq()
        return self.count
    
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
        if dome_controller.stop[0] == 1:
            self.buffer[1] = 0
        else: self.buffer[1] = 1
        #print(self.buffer)
        #self.dio.do_output(self.buffer, 1, 6)
        #self.dio.output_point(self.buffer, 1)
        print('do_output')
        #self.dome_limit()
        #pos_arcsec = self.dome_pos.dome_encoder_acq()
        #pos_arcsec = self.dome_pos.read_dome_enc()
        return
    
    def get_action(self):
        #ret = self.dio.di_check(1, 1)
        #ret = self.dio.input_point(1, 1)
        #print('l269', ret)
        if ret == 0:
            move_status = 'OFF'
        else:
            move_status = 'DRIVE'
        return move_status
    
    def get_door_status(self):
        """
        #ret = self.dio.di_check(2, 6)
        #ret = self.dio.input_point(2, 6)
        #print('l277', ret)
        if ret[0] == 0:
            right_act = 'OFF'
        else:
            right_act = 'DRIVE'
        
        if ret[1] == 0:
            if ret[2] == 0:
                right_pos = 'MOVE'
            else:
                right_pos = 'CLOSE'
        else:
            right_pos = 'OPEN'
        
        if ret[3] == 0:
            left_act = 'OFF'
        else:
            left_act = 'DRIVE'
        
        if ret[4] == 0:
            if ret[5] == 0:
                left_pos = 'MOVE'
            else:
                left_pos = 'CLOSE'
        else:
            left_pos = 'OPEN'
        """
        return [self.right_act, self.right_pos, self.left_act, self.left_pos]
        
    def get_memb_status(self):
        """
        #ret = self.dio.di_check(8, 3)
        #ret = self.dio.input_point(8, 3)
        if ret[0] == 0:
            memb_act = 'OFF'
        else:
            memb_act = 'DRIVE'
        
        if ret[1] == 0:
            if ret[2] == 0:
                memb_pos = 'MOVE'
            else:
                memb_pos = 'CLOSE'
        else:
            memb_pos = 'OPEN'
        """
        return [self.memb_act, self.memb_pos]
    
    def get_remote_status(self):
        """
        #ret = self.dio.di_check(11, 1)
        #ret = self.dio.input_point(11, 1)
        if ret[0] == 0:
            status = 'REMOTE'
        else:
            status = 'LOCAL'
        """
        return self.remote_status
    
    def error_check(self):
        #ret = self.dio.di_check(16, 6)
        #ret = self.dio.input_point(16, 6)
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
    
    def limit_check(self):
        while True:
            limit1 = self._limit_check()
            time.sleep(0.002)
            limit2 = self._limit_check()
            if limit1 == limit2:
                return limit1
            continue
        pass
        
    def _limit_check(self):
        """:
        #limit = self.dio.di_check(12, 4)
        #limit = self.dio.input_point(12, 4)
        #print(limit)
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
        """
        
    def dome_limit(self):
        limit = self.limit_check()
        if limit != 0:
            #self.dome_pos.dio.ctrl.set_counter(self.touchsensor_pos[limit-1]+self.dome_encoffset)
            self.dome_pos.dome_set_counter(self.touchsensor_pos[limit-1]+self.dome_encoffset)
            #print('!!!dome_pos_clear!!!')
        #print (limit)
        self.get_count()
        #print (self.count)
        return limit
    
    def get_domepos(self):
        self.limit = self.dome_limit()
        self.dome_enc = self.dome_pos.dome_encoder_acq()
        dome_enc_print = float(self.dome_enc)
        dome_enc_print = self.dome_enc/3600.
        #rospy.logwarn(dome_enc_print)
        f = open('./dome_enc1.txt', 'a')
        f.write(str(dome_enc_print)+'\n')
        f.close()
        return self.dome_enc
    
    def read_limit(self):
        return self.limit
    
    def start_status_check(self):
        #self.stop_status_flag = threading.Event()
        #self.status_thread = threading.Thread(target = self.status_check)
        #self.status_thread.start()
        th1 = threading.Thread(target = self.status_check)
        th1.setDaemon(True)
        th1.start()
        return
    
    def status_check(self):
        #while not self.stop_status_flag.is_set():
        while True:
            #ret1 = self.get_action()
            #ret2 = self.get_door_status()
            #ret3 = self.get_memb_status()
            #ret4 = self.get_remote_status()
            #ret5 = str(self.get_domepos())
            ret1 = self.move_status
            ret2 = [self.right_act, self.right_pos, self.left_act, self.left_pos]
            ret3 = [self.memb_act,self.memb_pos ]
            ret4 = self.remote_status
            ret5 = self.dome_enc
            #self.status_box = [ret1, ret2, ret3, ret4]###dome.py version
            self.status_box = [ret1, ret2[0], ret2[1], ret2[2], ret2[3], ret3[0], ret3[1], ret4, ret5]
            time.sleep(0.01)
        return
    
    def stop_status_check(self):
        self.stop_staus_flag.set()
        self.status_thread.join()
        return
    
    def read_count(self):
        return self.count
    
    def read_status(self):
        return self.status_box
    
    def read_domepos(self):
        return self.dome_enc

    ###ROS part

    ###start threading
    def start_thread_ROS(self):
        th = threading.Thread(target = self.pub_status)
        th.setDaemon(True)
        th.start()
        th2 = threading.Thread(target = self.act_dome)
        th2.setDaemon(True)
        th2.start()
        th3 = threading.Thread(target = self.stop_dome)
        th3.setDaemon(True)
        th3.start()

    ###set encoder az for dome tracking
    def set_enc_parameter(self, req):
        self.enc_az = req.enc_az
        return

    ###write command from ROS_controller.py
    def set_command(self, req):
        name = req.name
        value = req.value
        self.parameters[name] = value
        self.paralist.append(self.parameters[name])
        self.flag = 0
        print(name,value)
        return
    
    def set_az_command(self, req):
        self.parameter_az = req.value
        return

    ###function call to dome/memb action 
    def act_dome(self):
        while True:
            print('wait command...')
            if self.flag == 1:
                if self.paralist == []:
                    time.sleep(0.01)
                    continue
                else:
                    self.flag = 0
                    continue
            if self.paralist[0] == 'start':
                self.flag = 1
            elif self.paralist[0] == 'pass':
                time.sleep(0.01)
            elif self.paralist[0] == 'dome_open':
                self.dome_open()
                self.flag = 1
            elif self.paralist[0] == 'dome_close':
                self.dome_close()
                self.flag = 1
            elif self.paralist[0] == 'memb_open':
                self.memb_open()
                self.flag = 1
            elif self.paralist[0] == 'memb_close':
                self.memb_close()
                self.flag = 1
            elif self.paralist[0] == 'dome_move':
                sub3 = rospy.Subscriber('dome_move_az', Dome_msg, self.set_az_command)
                time.sleep(0.1)
                self.end_flag = False
                self.move(self.parameter_az)
                self.flag = 1
            elif self.paralist[0] == 'dome_tracking':
                self.end_flag = False
                self.move_track()
                self.flag = 1
                pass
            del self.paralist[0]
            time.sleep(0.01)
            continue

    def stop_dome(self):
        while True:
            if ('dome_stop' in self.paralist) == True:
                self.dome_stop()
                self.end_flag = True
                self.flag = 1
                print('!!!dome_stop!!!')
                self.paralist = ['start']
            elif ('dome_track_end' in self.paralist) == True:
                self.end_flag = True
                self.flag = 1
                print('dome track end')
                self.paralist = ['start']
            else:
                pass
            time.sleep(0.01)
            continue        


    ###publish status
    def pub_status(self):
        while True:
            pub = rospy.Publisher('status_dome', Status_dome_msg, queue_size=10, latch = True)
            s = Status_dome_msg()
            s.move_status = self.status_box[0]
            s.right_act = self.status_box[1]
            s.right_pos = self.status_box[2]
            s.left_act = self.status_box[3]
            s.left_pos = self.status_box[4]
            s.memb_act = self.status_box[5]
            s.memb_pos = self.status_box[6]
            s.remote_status = self.status_box[7]
            #s.status = self.status_box[:8]
            s.dome_enc = float(self.status_box[8])
            s.from_node = node_name
            s.timestamp = time.time()
            pub.publish(s)
            time.sleep(0.1)
        
        


def dome_client(host, port):
    client = pyinterface.server_client_wrapper.control_client_wrapper(dome_controller, host, port)
    return client

def dome_monitor_client(host, port):
    client = pyinterface.server_client_wrapper.monitor_client_wrapper(dome_controller, host, port)
    return client

def start_dome_server(port1 = 8007, port2 = 8008):
    dome = dome_controller()
    server = pyinterface.server_client_wrapper.server_wrapper(dome,'', port1, port2)
    server.start()
    return server


if __name__ == '__main__':
    rospy.init_node(node_name)
    d = dome_controller()
    d.start_thread_ROS()
    print('[ROS_dome.py] : START SUBSCRIBE')
    sub1 = rospy.Subscriber('status_encoder', Status_encoder_msg, d.set_enc_parameter)
    sub2 = rospy.Subscriber('dome_move', Dome_msg, d.set_command)
    rospy.spin()
    
