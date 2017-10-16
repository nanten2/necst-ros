import time
import math
import threading
#import pyinterface
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from necst.msg import Dome_msg
from necst.msg import Status_encoder_msg
from necst.msg import Status_dome_msg
#import dome_pos #DE
# antenna_enc #DE



class dome_controller(object):
    #speed = 3600 #[arcsec/sec]
    touchsensor_pos = [-391,-586,-780,-974,-1168,-1363,-1561,-1755,-1948,-2143, 0, -197]
    dome_encoffset = 10000
    buffer = [0,0,0,0,0,0]
    stop = [0]
    error = []
    count = 0
    status_box = []
    #dome_enc = 0
    limit = 0

    ###dome
    right_act = 'OFF'
    right_pos = 'CLOSE' 
    left_act = 'OFF'
    left_pos = 'CLOSE'
    ###memb
    memb_act = 'OFF'
    memb_pos = 'CLOSE'

    ###get_action/move_status
    move_status = 'OFF'

    ###remote/local
    remote_status = 'LOCAL'

    ###speed/turn
    speed = 'None'
    turn = 'None'

    dome_enc = '0'#[arcsec]
    status_box = ['00']*9

    ###encoder
    enc_az = '0'
    ###flag
    flag = 0

    ###parameters
    parameters = {
        'target_az':0,
        'flag':0,
        'command':'None',
        }
    
    def __init__(self):
        #self.enc = antenna_enc.enc_monitor_client('172.20.0.11',8002)
        #self.dome_pos = dome_pos.dome_pos_controller()
        #self.dio = pyinterface.create_gpg2000(5)
        #self.start_status_check()
        pass

    def start_thread_ROS(self):#for dummy
        th = threading.Thread(target = self.pub_status)
        th.setDaemon(True)
        th.start()
        th2 = threading.Thread(target = self.act_dome)
        th2.setDaemon(True)
        th2.start()
        th3 = threading.Thread(target = self.status_check)
        th3.setDaemon(True)
        th3.start()
        th4 = threading.Thread(target = self.calc_dome_pos)###for dummy
        th4.setDaemon(True)
        th4.start()
        return
        
        
    
    def start_thread(self):
        tlist = threading.enumerate()
        for i in tlist:
            n = i.getName()
            if n == "d_track":
                return
        self.end_track_flag = threading.Event()
        self.thread = threading.Thread(target = self.move_track, name = "d_track")
        self.thread.start()
        return
    
    def end_thread(self):
        try:
            self.end_track_flag.set()
            self.thread.join()
            buff = [0]
            #self.dio.do_output(buff, 2, 1)#sio
            return
        except:
            pass
    
    def move_track(self):
        #ret = self.read_domepos()
        while self.end_flag:
            #ret = self.enc.read_azel()
            enc_az = float(self.enc_az)
            #ret[0] = ret[0]/3600. # ret[0] = antenna_az
            enc_az = enc_az/3600.
            #dome_az = self.read_domepos()
            dome_az = float(self.dome_enc)
            dome_az = dome_az/3600.
            #self.dome_limit()
            if math.fabs(enc_az - dome_az) >= 2.0:
                self.move(enc_az)
            time.sleep(0.01)
    
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
    
    def move(self, dist):
        #pos_arcsec = self.read_domepos()
        pos_arcsec = float(self.dome_enc)
        pos = pos_arcsec/3600.
        pos = pos % 360.0
        dist = float(dist)
        dist = dist % 360.0
        diff = dist - pos
        dir = diff % 360.0
        
        if pos == dist: return
        if diff < 0:
            dir = dir*(-1)
        if dir < 0:
            if abs(dir) <= 180:
                turn = 'right'
                self.turn = 'right'
            else:
                turn = 'left'
                self.turn = 'left'
        else:
            if abs(dir) >= 180:
                turn = 'left'
                self.turn = 'left'
            else:
                turn = 'right'
                self.turn = 'right'
        if abs(dir) < 5.0 or abs(dir) > 355.0 :
            speed = 'low'
            self.speed = 'low'
        elif abs(dir) > 15.0 and abs(dir) < 345.0:
            speed = 'high'
            self.speed = 'high'
        else:
            speed = 'mid'
            self.speed = 'mid'
        if dir != 0:
            #global buffer#DE
            self.buffer[1] = 1
            #self.do_output(turn, speed)
            while dir != 0:
                #pos_arcsec = self.read_domepos()
                pos_arcsec = float(self.dome_enc)
                pos = pos_arcsec/3600.
                pos = pos % 360.0
                dist = dist % 360.0
                diff = dist - pos
                dir = diff % 360.0
                #print(pos,dist,diff,dir)
                if dir <= 0.5:
                    dir = 0
                else:
                    if abs(dir) < 5.0 or dir > 355.0:
                        speed = 'low'
                        self.speed = 'low'
                    elif abs(dir) > 20.0 and abs(dir) < 340.0:
                        speed = 'high'
                        self.speed = 'high'
                    else:
                        speed = 'mid'
                        self.speed = 'mid'
                    #self.do_output(turn, speed)
                time.sleep(0.1)
        self.dome_stop()
        self.speed = 'None'
        return
    
    def dome_stop(self):###need be improved
        buff = [0]
        #self.dio.do_output(buff, 2, 1)
        return
    
    def dome_open(self):
        if self.right_pos == 'OPEN' and self.left_pos == 'OPEN':
            pass
        else:
            self.right_pos = 'MOVE'
            self.left_pos = 'MOVE'
            time.sleep(4)
            self.right_pos = 'OPEN'
            self.left_pos = 'OPEN'
        
        '''
        ret = self.get_door_status()
        if ret[1] != 'OPEN' and ret[3] != 'OPEN':
            buff = [1, 1]
            self.dio.do_output(buff, 5, 2)
            d_door = self.get_door_status()
            while ret[1] != 'OPEN':
                time.sleep(5)
                ret = self.get_door_status()
        buff = [0, 0]
        self.dio.do_output(buff, 5, 2)
        '''
        return

    
    def dome_close(self):
        if self.right_act == 'CLOSE' and self.left_act == 'CLOSE':
            pass
        else:
            self.right_pos = 'MOVE'
            self.left_pos = 'MOVE'
            time.sleep(4)
            self.right_pos = 'CLOSE'
            self.left_pos = 'CLOSE'
        """
        ret = self.get_door_status()
        if ret[1] != 'CLOSE' and ret[3] != 'CLOSE':
            buff = [0, 1]
            self.dio.do_output(buff, 5, 2)
            while ret[1] != 'CLOSE':
                time.sleep(5)
                ret = self.get_door_status()
        buff = [0, 0]
        self.dio.do_output(buff, 5, 2)
        """
        return
    
    def memb_open(self):
        print('memb_open')
        if self.memb_pos == 'OPEN':
            pass
        else:
            self.memb_pos = 'MOVE'
            time.sleep(4)
            self.memb_pos = 'OPEN'
        """
        ret = self.get_memb_status()
        if ret[1] != 'OPEN':
            buff = [1, 1]
            self.dio.do_output(buff, 7, 2)
            while ret[1] != 'OPEN':
                time.sleep(5)
                ret = self.get_memb_status()
        buff = [0, 0]
        self.dio.do_output(buff, 7, 2)
        """
        return
    
    def memb_close(self):
        if self.memb_pos == 'CLOSE':
            pass
        else:
            self.memb_pos = 'MOVE'
            time.sleep(4)
            self.memb_pos = 'CLOSE'
        """
        ret = self.get_memb_status()
        if ret[1] != 'CLOSE':
            buff = [0, 1]
            self.dio.do_output(buff, 7, 2)
            while ret[1] != 'CLOSE':
                time.sleep(5)
                ret = self.get_memb_status()
        buff = [0, 0]
        self.dio.do_output(buff, 7, 2)
        """
        return
    
    def emergency_stop(self):
        global stop
        dome_controller.stop = [1]
        #self.pos.dio.do_output(self.stop, 11, 1)
        self.print_msg('!!EMERGENCY STOP!!')
        return
    
    def dome_fan(self, fan):
        if fan == 'on':
            fan_bit = [1, 1]
            #self.dio.do_output(fan_bit, 9, 2)
        else:
            fan_bit = [0, 0]
            #self.dio.do_output(fan_bit, 9, 2)
        return
    
    def get_count(self):
        #self.count = self.dome_pos.dome_encoder_acq()
        #return self.count
        pass
    """
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
        self.dio.do_output(self.buffer, 1, 6)
        return
    
    
    def get_action(self):
        ret = self.dio.di_check(1, 1)
        if ret == 0:
            move_status = 'OFF'
        else:
            move_status = 'DRIVE'
        return move_status
    
    def get_door_status(self):
        ret = self.dio.di_check(2, 6)
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
        return [right_act, right_pos, left_act, left_pos]
        
    def get_memb_status(self):
        ret = self.dio.di_check(8, 3)
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
        return [memb_act, memb_pos]
    
    def get_remote_status(self):
        ret = self.dio.di_check(11, 1)
        if ret[0] == 0:
            status = 'REMOTE'
        else:
            status = 'LOCAL'
        return status
    """

    """
    
    def error_check(self):
        ret = self.dio.di_check(16, 6)
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
    """

    """
    
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
        limit = self.dio.di_check(12, 4)
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
    
    def dome_limit(self):
        limit = self.limit_check()
        if limit != 0:
            self.dome_pos.dome_set_counter(self.touchsensor_pos[limit-1]+self.dome_encoffset)
        self.get_count()
        return limit
    
    def get_domepos(self):
        self.limit = self.dome_limit()
        self.dome_enc = self.dome_pos.dome_encoder_acq()
        return self.dome_enc
    
    def read_limit(self):
        return self.limit
    """
    
    def start_status_check(self):
        self.stop_status_flag = threading.Event()
        self.status_thread = threading.Thread(target = self.status_check)
        self.status_thread.start()
        return
    
    def status_check(self):
        #while not self.stop_status_flag.is_set():
        while True:
            #ret1 = self.get_action()
            #ret2 = self.get_door_status()
            #ret3 = self.get_memb_status()
            #ret4 = self.get_remote_status()
            #ret5 = self.get_domepos()
            self.status_box = [self.move_status, self.right_act, self.right_pos, self.left_act, self.left_pos, self.memb_act, self.memb_pos, self.remote_status, self.dome_enc]
            time.sleep(0.1)

    def pub_status(self):
        while True:
            pub = rospy.Publisher('status_dome', Status_dome_msg, queue_size=10, latch = True)
            s = Status_dome_msg()
            s.status = self.status_box
            #print(self.status_box)
            #rospy.loginfo('publish status')
            pub.publish(s)
            time.sleep(0.3)
            
    
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

    def act_dome(self):
        while True:
            if self.flag == 1:
                time.sleep(1)
                continue
            if self.parameters['command'] == 'pass':
                time.sleep(1)
            elif self.parameters['command'] == 'dome_open':
                self.dome_open()
                self.flag = 1
            elif self.parameters['command'] == 'dome_close':
                self.dome_close()
                self.flag = 1
            elif self.parameters['command'] == 'memb_open':
                self.memb_open()
                self.flag = 1
            elif self.parameters['command'] == 'memb_close':
                self.memb_close()
                self.flag = 1
            elif self.parameters['command'] == 'dome_move':
                self.move(self.parameters['target_az'])
                self.flag = 1
            elif self.parameters['command'] == 'dome_stop':
                self.dome_stop()
                self.end_flag = False
                self.flag = 1
            elif self.parameters['command'] == 'dome_tracking':
                self.end_flag = True
                self.move_track()
                self.flag = 1
                pass
            time.sleep(1)
            continue
        
    def set_enc_parameter(self, req):
        self.enc_az = req.enc_az
        #rospy.loginfo('set_enc_parameter')###for check
        return

    def set_parameter(self, req):
        a = req.name
        b = req.value
        rospy.logwarn(a)
        rospy.logwarn(b)
        self.parameters[a] = b
        self.flag = 0 
        rospy.logwarn('set_parameter')###for check
        

    def calc_dome_pos(self):
        while True:
            #print('calc_pos',self.turn, self.speed)
            if self.speed == 'high':
                speed = 10000
            elif self.speed == 'mid':
                speed = 3000
            elif self.speed == 'low':
                speed = 1000
            else:
                speed = 0
            dome_enc = float(self.dome_enc)
            dome_enc = dome_enc%1296000 # 360*3600
            if self.turn == 'right':
                dome_enc += speed
            elif self.turn == 'left':
                dome_enc += speed
            self.dome_enc = str(dome_enc)
            #print(self.dome_enc)
            dome_enc = dome_enc/3600.
            rospy.loginfo(dome_enc)
            time.sleep(0.9)



if __name__ == '__main__':
    rospy.init_node('dome_dummy')
    d = dome_controller()
    d.start_thread_ROS()
    sub1 = rospy.Subscriber('dome_move', Dome_msg, d.set_parameter)
    #sub2 = rospy.Subscriber('emergency', String, d.emergency)
    sub3 = rospy.Subscriber('status_encoder', Status_encoder_msg, d.set_enc_parameter)
    #sub4 = rospy.Subscriber('dome_move_stop', Bool, d.stop_flag_set)
    #sub5 = rospy.Subscriber('dome_enc', String, d.set_domeenc)
    rospy.spin()
