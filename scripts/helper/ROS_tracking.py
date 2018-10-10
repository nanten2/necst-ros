#!/usr/bin/env python3

import time
import threading
import rospy
from necst.msg import Status_antenna_msg
from necst.msg import Status_encoder_msg
from necst.msg import List_coord_msg
from necst.msg import Bool_necst

node_name = 'tracking'

class tracking_check(object):
    enc_param = {
        'enc_az' : 0,
        'enc_el' : 0
        }
    antenna_param = {
        'command_az' : 200,
        'command_el' : 200
        }
    """
    coordinate_param = {
        "x_list" : "",
        "y_list" : "",
        "off_x" : "",
        "off_y" : "",
        "coord" : ""
        }
    """
    coordinate_param = ""
    tracking = False
    coord_flag = False
    command_flag = False
    same_coord_flag = False
    receive_flag1 = False
    
    list_coord = ''
    before_x = -10
    before_y = -10
    same_azel_list_flag = False
    
    def __init__(self):
        self.start_thread()
        pass

    def start_thread(self):
        th = threading.Thread(target = self.pub_tracking)
        th.setDaemon(True)
        th.start()
        check = threading.Thread(target = self.check_track)
        check.setDaemon(True)
        check.start()        
        

    def set_ant_param(self, req):
        if abs(self.antenna_param['command_az']-req.command_az) > 3 or abs(self.antenna_param['command_az']-req.command_az) > 3:
            self.command_flag = True
            #rospy.logwarn("change command")
        else:
            pass
        self.antenna_param['command_az'] = req.command_az
        self.antenna_param['command_el'] = req.command_el
        return

    def set_enc_param(self, req):
        self.enc_param['enc_az'] = req.enc_az
        self.enc_param['enc_el'] = req.enc_el
        return

    def set_list_param(self, req):
        # check new list
        tmp_list = self.coordinate_param        
        try:
            tmp_list.x_list = [round(i,1) for i in tmp_list.x_list]
            tmp_list.y_list = [round(i,1) for i in tmp_list.y_list]           
            tmp_list.time_list = []
            tmp_list.timestamp = 0.0
        except:
            print("First receive !!")
        tmp_req = req
        tmp_req.x_list = [round(i,1) for i in tmp_req.x_list]
        tmp_req.y_list = [round(i,1) for i in tmp_req.y_list]           
        tmp_req.timestamp = 0.0
        tmp_req.time_list = []

        # parameter change
        if tmp_list == tmp_req:
            rospy.logerr("same command !!")
            if tmp_req.coord == "altaz":
                self.same_coord_flag = True
            else:
                self.same_coord_flag = False
        else:
            self.same_coord_flag = False
        self.coordinate_param = req
        self.coord_flag = True
        self.receive_flag1 = True        
        return

    def check_track(self):
        track_count = 0
        pub = rospy.Publisher("move_stop", Bool_necst, queue_size = 1)
        while not self.coordinate_param:
            print("wait command...")
            time.sleep(0.1)
        while not rospy.is_shutdown():
            #current_coordinate = self.coordinate_param
            command_az = self.antenna_param['command_az']
            command_el = self.antenna_param['command_el']
            command_coord = self.coordinate_param.coord   
            #print(current_coordinate,command_az, command_el)

            if not self.coord_flag:
                print("coordinate check")
                time.sleep(0.1)
                continue
            else:
                print("coordinate clear")
                self.tarcking = False                
                pass

            if self.command_flag == False or self.same_coord_flag == True:
                print("same command")
                time.sleep(0.1)
                continue                

            """ start checking track """
            enc_az = self.enc_param['enc_az']
            enc_el = self.enc_param['enc_el']            
            
            if command_az < 0:
                command_az += 360*3600
            if enc_az <0:
                enc_az += 360*3600
            d_az = abs(command_az - enc_az)
            d_el = abs(command_el - enc_el)

            list_coord = self.list_coord
                
            if d_az <= 3 and d_el <=3:
                track_count += 1
            else:
                track_count = 0
            if track_count >= 5:# if tracking is True for 0.5[se]
                self.tracking = True
                if command_coord == "altaz":
                    pub.publish(False, 'ROS_tracking.py', time.time())

                self.coord_flag = False
                self.command_flag = False
                track_count = 0
            else:
                self.tracking = False
            
            time.sleep(0.1)
            rospy.loginfo('tracking : %s'%self.tracking)
        return self.tracking

    def pub_tracking(self):
        pub = rospy.Publisher('tracking_check', Bool_necst, queue_size = 10, latch = True)
        track_status = Bool_necst()
        while not rospy.is_shutdown():
            track_status.data = self.tracking
            track_status.from_node = node_name
            track_status.timestamp = time.time()
            pub.publish(track_status)
            time.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node(node_name)
    t = tracking_check()
    sub1 = rospy.Subscriber('status_antenna', Status_antenna_msg, t.set_ant_param)
    sub2 = rospy.Subscriber('status_encoder',Status_encoder_msg, t.set_enc_param)
    sub3 = rospy.Subscriber('wc_list',List_coord_msg, t.set_list_param, queue_size = 1)
    rospy.spin()
