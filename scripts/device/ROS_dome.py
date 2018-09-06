#!/usr/bin/env python3

import time
import threading
import sys
sys.path.append("/home/necst/ros/src/necst/lib/")
sys.path.append("/home/necst/ros/src/necst/lib/device/")
import dome_pos
import dome_device
#ROS
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import rospy
from necst.msg import Status_encoder_msg
from necst.msg import Status_dome_msg
from necst.msg import Dome_msg
from necst.msg import Bool_necst

node_name = "dome"
import topic_status
deco = topic_status.deco(node_name)

class dome_controller(object):
    touchsensor_pos = [-391,-586,-780,-974,-1168,-1363,-1561,-1755,-1948,-2143, 0, -197]
    dome_encoffset = 10000
    count = 0
    limit = 0
    dome_enc = 0

    ###ROS_dome.py parameter
    enc_az = 0###antenna az for dome tracking

    parameters = {
        'command':0
        }
    paralist = []
    parameter_az = 0
    ###command flags
    end_flag = True

    def __init__(self):
        self.dome_pos = dome_pos.dome_pos_controller()
        self.dev = dome_device.dome_device()
        self.read_init_domepos()
        self.start_status_check()
        pass

    def read_init_domepos(self):
        try:
            a = open('dome_enc.txt', 'r')
            self.dome_enc = float(a.read())
            print(self.dome_enc)
        except:
            pass
        

    def start_thread(self):
        tlist = threading.enumerate()
        for i in tlist:
            n = i.getName()
            if n == "d_track":
                return
        self.end_track_flag = threading.Event()
        self.thread = threading.Thread(target = self.dev.move_track, name = "d_track")
        self.thread.start()
        return
    
    def end_thread(self):
        try:
            self.end_track_flag.set()
            self.thread.join()
            buff = [0]
            self.dev.dio.output_point(buff, 2)
            return
        except:
            pass

    def test(self, num): #for track_test
        self.start_thread()
        time.sleep(num)
        self.end_thread()
        return

    def move_org(self):
        dist = 90
        pos = self.dome_enc
        dir = self.dev.move(dist, pos)#move_org
        while not dir <= 0.5:
            pos = self.dome_enc
            dir = self.dev.move(dist, pos)
        return

    def con_move_track(self):
        print("dome_tracking start", self.end_flag)
        while not self.end_flag:
            dome_az = self.dome_enc
            #print('dome_az',dome_az)
            #print('enc_az',self.enc_az)
            dir = self.dev.move_track(self.enc_az, dome_az)
            if dir <= 1.5 or dir >= 358.5:
                self.dev.dome_stop()
        if self.end_flag:
            self.dev.dome_stop()
            while "dome_move" in self.paralist: # 0904
                self.paralist.remove("dome_move")
            while "dome_tracking" in self.paralist:
                self.paralist.remove("dome_tracking")
        return

    def con_move(self, dist):
        while not self.end_flag:
            pos = self.dome_enc
            dir = self.dev.move(dist, pos)
            print(dir,'<1.5 => stop')
            if dir <= 1.5 or dir >= 358.5:
                self.dev.dome_stop()
                self.paralist.remove("dome_move")
                break
        if self.end_flag:
            self.dev.dome_stop()
            while "dome_move" in self.paralist:
                self.paralist.remove("dome_move")
        return
    
    def get_count(self):
        self.count = self.dome_pos.dome_encoder_acq()
        return
    
    def limit_check(self):
        while True:
            limit1 = self.dev._limit_check()
            time.sleep(0.002)
            limit2 = self.dev._limit_check()
            if limit1 == limit2:
                return limit1
            continue
        pass

    def dome_limit(self):
        limit = self.limit_check()
        if limit != 0:
            self.dome_pos.dome_set_counter(self.touchsensor_pos[limit-1]+self.dome_encoffset)
        self.get_count()
        return limit
    
    def get_domepos(self):
        self.limit = self.dome_limit()
        self.dome_enc = self.dome_pos.dome_encoder_acq()
        dome_enc_print = float(self.dome_enc)
        dome_enc_print = self.dome_enc/3600.
        f = open('./dome_enc.txt', 'a')
        f.write(str(dome_enc_print)+'\n')
        f.close()
        return 
    
    def start_status_check(self):
        th1 = threading.Thread(target = self.status_check)
        th1.setDaemon(True)
        th1.start()
        return
    
    def status_check(self):
        while not rospy.is_shutdown():
            self.dev.get_action()
            self.dev.get_door_status()
            self.dev.get_memb_status()
            self.dev.get_remote_status()
            self.get_domepos()
            time.sleep(0.1)

    ###ROS part

    ###start threading
    def start_thread_ROS(self):
        th = threading.Thread(target = self.pub_status)
        th.setDaemon(True)
        th.start()
        th2 = threading.Thread(target = self.dome_OC)
        th2.setDaemon(True)
        th2.start()
        th3 = threading.Thread(target = self.memb_OC)
        th3.setDaemon(True)
        th3.start()
        th4 = threading.Thread(target = self.act_dome)
        th4.setDaemon(True)
        th4.start()
        th5 = threading.Thread(target = self.stop_dome)
        th5.setDaemon(True)
        th5.start()



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
        print(name,value)
        return

    def set_az_command(self, req):
        self.parameter_az = req.value
        return

    ###function call to dome/memb action 
    def dome_OC(self):
        while not rospy.is_shutdown():
            if self.paralist == []:
                time.sleep(1)
                continue
            elif "dome_open" in self.paralist and "dome_close" in self.paralist:
                if self.paralist.index("dome_open") < self.paralist.index("dome_close"):
                    self.dev.dome_open()
                    self.paralist.remove("dome_open")
                else:
                    self.dev.dome_close()
                    self.paralist.remove("dome_close")
            elif "dome_open" in self.paralist:
                self.dev.dome_open()
                self.paralist.remove("dome_open")
            elif "dome_close" in self.paralist:
                self.dev.dome_close()
                self.paralist.remove("dome_close")
            else:
                time.sleep(1)
                continue
            time.sleep(1)
            continue

    def memb_OC(self):
        while not rospy.is_shutdown():
            if self.paralist == []:
                time.sleep(1)
                continue
            elif "memb_open" in self.paralist and "memb_close" in self.paralist:
                if self.paralist.index("memb_open") < self.paralist.index("memb_close"):
                    self.dev.memb_open()
                    self.paralist.remove("memb_open")
                else:
                    self.dev.memb_close()
                    self.paralist.remove("memb_close")
            elif "memb_open" in self.paralist:
                self.dev.memb_open()
                self.paralist.remove("memb_open")
            elif "memb_close" in self.paralist:
                self.dev.memb_close()
                self.paralist.remove("memb_close")
            else:
                time.sleep(1)
                continue
            time.sleep(1)
            continue

    def act_dome(self):
        while not rospy.is_shutdown():
            if self.paralist == []:
                time.sleep(1)
                continue
            elif "dome_move" in self.paralist and "dome_tracking" in self.paralist:
                if self.paralist.index("dome_move") < self.paralist.index("dome_tracking"):
                    sub3 = rospy.Subscriber('dome_move_az', Dome_msg, self.set_az_command)
                    time.sleep(0.1)
                    self.end_flag = False
                    self.con_move(self.parameter_az)
                else:
                    self.end_flag = False
                    self.con_move_track()
            elif "dome_move" in self.paralist:
                sub3 = rospy.Subscriber('dome_move_az', Dome_msg, self.set_az_command)
                time.sleep(0.1)
                self.end_flag = False
                self.con_move(self.parameter_az)
            elif "dome_tracking" in self.paralist:
                self.end_flag = False
                self.con_move_track()
            else:
                time.sleep(1)
                continue
            time.sleep(1)
            continue

    def stop_dome(self):
        while not rospy.is_shutdown():
            if "dome_stop" in self.paralist:
                self.dev.dome_stop()
                self.end_flag = True
                print('!!!dome_stop!!!')
                self.paralist.remove("dome_stop")
            elif "dome_track_end" in self.paralist:
                self.end_flag = True
                print("dome track end")
                self.paralist.remove("dome_track_end")
            elif "pass" in self.paralist:
                time.sleep(1)
                self.paralist.remove("pass")
            else:
                pass
            time.sleep(1)
            continue

    ###publish status
    @deco
    def pub_status(self):
        track_pub = rospy.Publisher('dome_track_flag', Bool_necst, queue_size=1)
        while not rospy.is_shutdown():
            pub = rospy.Publisher('status_dome', Status_dome_msg, queue_size=10, latch = True)
            s = Status_dome_msg()
            s.move_status = self.dev.move_status
            s.right_act = self.dev.right_act
            s.right_pos = self.dev.right_pos
            s.left_act = self.dev.left_act
            s.left_pos = self.dev.left_pos
            s.memb_act = self.dev.memb_act
            s.memb_pos = self.dev.memb_pos
            s.remote_status = self.dev.remote_status
            s.dome_enc = float(self.dome_enc)
            s.from_node = node_name
            s.timestamp = time.time()
            pub.publish(s)
            if self.end_flag:
                track_pub.publish(False, node_name, time.time())
            elif not self.end_flag:
                track_pub.publish(True, node_name, time.time())
            else:
                print("where track flag???")
            time.sleep(0.1)
        
if __name__ == '__main__':
    rospy.init_node(node_name)
    d = dome_controller()
    d.start_thread_ROS()
    print('[ROS_dome.py] : START SUBSCRIBE')
    sub1 = rospy.Subscriber('status_encoder', Status_encoder_msg, d.set_enc_parameter)
    sub2 = rospy.Subscriber('dome_move', Dome_msg, d.set_command)
    rospy.spin()
    
