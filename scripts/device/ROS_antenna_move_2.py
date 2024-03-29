#!/usr/bin/env python3

import sys

sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import rospy
import numpy
import time
import threading

sys.path.append("/home/necst/ros/src/necst/lib/device")
sys.path.append("/home/amigos/ros/src/necst/lib/device")
from datetime import datetime as dt

# ROS/import field
# ----------------
from necst.msg import Status_antenna_msg
from necst.msg import List_coord_msg
from necst.msg import Status_encoder_msg
from necst.msg import Bool_necst
from necst.msg import Status_pid_msg

node_name = "antenna_move"
mode = sys.argv[1]  #'Actual/Simulator'
if mode in ["Actual", "Simulator"]:
    import antenna_device
elif mode == "Actual_reverse":
    import antenna_device_reverse as antenna_device
else:
    rospy.logwarn("Launch this node with launch file")


class antenna_move(object):

    # initial parameter
    # -----------------
    parameters = {
        "az_list": [],
        "el_list": [],
        "start_time_list": [],
    }

    enc_parameter = {"az_enc": 0, "el_enc": 0}

    stop_flag = True  # False
    error = False  # False = ok
    emergency_flag = False
    limit_az = True  ###(True/False = okay/limit)
    limit_el = True
    command_az = 0
    command_el = 0

    tracking_status = False
    list_coord = ""
    timestamp = ""

    """
    ###for module
    az_rate_d = 0
    el_rate_d = 0

    m_stop_rate_az = 0
    m_stop_rate_el = 0
    """
    rate_az = 0
    rate_el = 0
    target_arcsec_az = 0
    target_arcsec_el = 0
    enc_arcsec_az = 0
    enc_arcsec_el = 0
    pre_hensa_az = 0
    pre_hensa_el = 0
    ihensa_az = 0
    ihensa_el = 0
    enc_before_az = 0
    enc_before_el = 0
    p_az = 0
    p_el = 0
    i_az = 0
    i_el = 0
    d_az = 0
    d_el = 0
    t_now = 0
    t_past = 0

    MOTOR_SAMPLING = 10  # memo
    mdt = MOTOR_SAMPLING / 1000.0

    # error_box = [0]*32###0921

    def __init__(self):
        if mode == "Simulator":
            self.dev = antenna_device.antenna_device(simulator=True)
        else:
            self.dev = antenna_device.antenna_device()
        self.start_time = time.time()
        self.node_status = "node_start[ROS_antenna_move]"
        pass

    def start_thread(self):
        th = threading.Thread(target=self.act_azel)
        th.setDaemon(True)
        th.start()
        th2 = threading.Thread(target=self.pub_status)
        th2.setDaemon(True)
        th2.start()
        th3 = threading.Thread(target=self.pub_status_pid)
        th3.setDaemon(True)
        th3.start()

    def set_tracking(self, req):
        self.tracking_status = req.data

    def set_parameter(self, req):
        if self.stop_flag and req.timestamp == self.timestamp:
            self.parameters = {"az_list": [], "el_list": [], "start_time_list": []}
            return
        elif req.timestamp == self.timestamp and self.parameters["az_list"] == []:
            return
        else:
            self.stop_flag = False
            self.timestamp = req.timestamp

        """
        DESCRIPTION
        ===========
        This function recieves azel_list and start_time from publisher in ROS_antenna.py 
        """
        rospy.loginfo("len {} ".format(len(self.parameters["az_list"])))
        if not req.time_list:
            return
        else:
            pass
        self.list_coord = req.coord
        if not self.stop_flag and self.start_time < req.time_list[0]:
            print("st,ct", self.stop_flag, self.start_time, req.time_list[0])
            if self.parameters["start_time_list"] != []:  ##combine_list
                time_len = len(self.parameters["start_time_list"])
                for i in range(time_len):
                    __tmp = self.parameters
                    if req.time_list[0] < __tmp["start_time_list"][-1]:
                        del __tmp["az_list"][-1]
                        del __tmp["el_list"][-1]
                        del __tmp["start_time_list"][-1]
                        self.parameters = __tmp
                    else:
                        break
            else:
                pass

            # _tmp = self.parameters
            # if len(_tmp['start_time_list']) > 100:
            # now = time.time()
            # if _tmp['start_time_list'][99] < now:
            # del _tmp['az_list'][:98]
            # del _tmp['el_list'][:98]
            # del _tmp['start_time_list'][:98]
            # self.parameters = _tmp
            # else:
            # pass

            tmp_ = self.parameters
            tmp_["az_list"].extend(req.x_list)
            tmp_["el_list"].extend(req.y_list)
            tmp_["start_time_list"].extend(req.time_list)
            self.parameters = tmp_
        else:
            pass
        return

    def set_enc_parameter(self, req):
        """
        DESCRIPTION
        ===========
        This function recieves encoder parameter(antenna's Az and El) from publisher in ROS_encoder.py
        This parameter is needed for PID control
        """
        self.enc_parameter["az_enc"] = req.enc_az
        self.enc_parameter["el_enc"] = req.enc_el
        return

    def limit_check(self, az, el):
        """
        DESCRIPTION
        ===========
        This function checks limit azel
        """
        if az > 240 * 3600.0 or az < -240 * 3600.0:  # 240?
            self.stop_flag = True
            print("!!!target az limit!!! : ", az)
            rospy.logwarn("!!!limit az!!!")
            self.error = True
            return True

        if el > 89 * 3600.0 or el < 0:
            self.stop_flag = True
            print("!!!target el limit!!! : ", el)
            rospy.logwarn("!!!limit el!!!")
            self.error = True
            return True

        else:
            return False

    def time_check(self, st):
        ct = time.time()

        if ct - st[-1] >= 0:
            rospy.loginfo("!!!azel_list is end!!!")
            # self.stop_flag = True
            self.dev.command_az_speed = 0
            self.dev.command_el_speed = 0
            time.sleep(0.25)
            return
        return ct

    def comp(self):
        """
        DESCRIPTION
        ===========
        This function determine target Az and El from azel_list
        """
        param = self.parameters
        st = param["start_time_list"]
        # rospy.loginfo(str(dt.utcnow()))
        # rospy.loginfo("start time : st")
        # rospy.loginfo(str(st))
        if st == []:
            return

        ct = self.time_check(st)
        if ct == None or param["az_list"] == []:
            return

        else:
            try:
                num = numpy.where(numpy.array(st) > ct)[0][0]
                # rospy.loginfo(str(dt.utcnow()))
                # rospy.loginfo("num + ct")
                # rospy.loginfo(str(num))
                # rospy.loginfo(str(ct))
                if len(param["az_list"]) > num:
                    if num == 0:
                        az_1 = az_2 = param["az_list"][num]
                        el_1 = el_2 = param["el_list"][num]
                        st2 = st[num]
                    else:
                        az_1 = param["az_list"][num - 1]
                        az_2 = param["az_list"][num]
                        el_1 = param["el_list"][num - 1]
                        el_2 = param["el_list"][num]
                        st2 = st[num - 1]
                else:
                    return
                # rospy.loginfo('len{}num{}'.format(len(param['az_list']), num))
                return (az_1, az_2, el_1, el_2, st2)
            except Exception as e:
                # rospy.loginfo(str(dt.utcnow()))
                rospy.logerr(e)
                return

    def act_azel(self):
        ###for azel_move check
        command_az_before = ""
        command_el_before = ""
        ###
        while True:
            if self.stop_flag or self.parameters["az_list"] == []:
                print("stop_flag")
                self.dev.emergency_stop()
                self.dev.command_az_speed = 0
                self.dev.command_el_speed = 0
                time.sleep(1)
                continue

            if self.emergency_flag:
                self.dev.emergency_stop()
                time.sleep(0.1)
                continue

            ret = self.comp()
            # print(ret)
            # rospy.loginfo(str(dt.utcnow()))
            # rospy.loginfo("act:start_time")
            # rospy.loginfo("act:current_time")
            # rospy.loginfo(str(time.time()))
            if ret == None:
                time.sleep(0.1)
                continue
            else:
                hensa_az = ret[1] - ret[0]
                hensa_el = ret[3] - ret[2]
                current_time = time.time()
                start_time = ret[4]
                tar_az = ret[0] + hensa_az * (current_time - start_time) * 10
                tar_el = ret[2] + hensa_el * (current_time - start_time) * 10

                tar_az_ = ret[0] + hensa_az * (current_time - start_time + 0.0665) * 10
                tar_el_ = ret[2] + hensa_el * (current_time - start_time + 0.0665) * 10

                if self.limit_check(tar_az_, tar_el_):
                    print(tar_az_, tar_el_)
                    time.sleep(0.1)
                    continue

                self.command_az = tar_az
                self.command_el = tar_el

                b_time = time.time()

                ret0 = self.dev.move_azel(
                    tar_az,
                    tar_el,
                    self.enc_parameter["az_enc"],
                    self.enc_parameter["el_enc"],
                )
                ret = self.dev.move_azel(
                    tar_az_,
                    tar_el_,
                    self.enc_parameter["az_enc"],
                    self.enc_parameter["el_enc"],
                )

                self.rate_az = ret[0]
                self.p_az = ret[1]
                self.i_az = ret[2]
                self.d_az = ret[3]
                self.target_arcsec_az = ret0[4]
                self.enc_arcsec_az = ret[5]
                self.pre_hensa_az = ret[6]
                self.ihensa_az = ret[7]
                self.enc_before_az = ret[8]
                self.rate_el = ret[9]
                self.p_el = ret[10]
                self.i_el = ret[11]
                self.d_el = ret[12]
                self.target_arcsec_el = ret0[13]
                self.enc_arcsec_el = ret[14]
                self.pre_hensa_el = ret[15]
                self.ihensa_el = ret[16]
                self.enc_before_el = ret[17]
                self.t_now = ret[18]
                self.t_past = ret[19]

                interval = time.time() - b_time
                if interval <= 0.01:
                    time.sleep(0.01 - interval)
                    pass
        return

    def stop_move(self, req):
        if time.time() - self.start_time < 1:
            return

        rospy.loginfo("***subscribe move stop***")
        self.stop_flag = True
        self.parameters["az_list"] = []
        self.parameters["el_list"] = []
        self.parameters["start_time_list"] = []
        return

    def emergency(self, req):
        if req.data:
            self.emergency_flag = True
            rospy.logwarn("!!!emergency!!!")
            rospy.logwarn("!!!stop azel velocity =>0!!!")
            self.dev.emergency_stop()
            rospy.logwarn("!!!exit ROS_antenna.py!!!")
            rospy.signal_shutdown("emergency")
            sys.exit()

    def pub_error(self):
        pub = rospy.Publisher("error", Bool_necst, queue_size=1, latch=True)
        error = Bool_necst()
        error.data = self.error
        error.from_node = node_name
        error.timestamp = time.time()
        pub.publish(error)

    def pub_status(self):
        pub = rospy.Publisher(
            "status_antenna", Status_antenna_msg, queue_size=1, latch=True
        )
        status = Status_antenna_msg()
        while not rospy.is_shutdown():
            # publisher
            # ---------
            status.limit_az = self.limit_az
            status.limit_el = self.limit_el
            status.command_az = self.command_az
            status.command_el = self.command_el
            status.emergency = self.emergency_flag
            status.command_azspeed = self.dev.command_az_speed
            status.command_elspeed = self.dev.command_el_speed
            status.node_status = self.node_status
            status.from_node = node_name
            status.timestamp = time.time()
            pub.publish(status)
            time.sleep(0.001)
            continue

    def pub_status_pid(self):
        pub = rospy.Publisher("status_pid", Status_pid_msg, queue_size=1, latch=True)
        msg = Status_pid_msg()
        while not rospy.is_shutdown():
            msg.rate_az = self.rate_az
            msg.p_az = self.p_az
            msg.i_az = self.i_az
            msg.d_az = self.d_az
            msg.target_arcsec_az = self.target_arcsec_az
            msg.enc_arcsec_az = self.enc_arcsec_az
            msg.pre_hensa_az = self.pre_hensa_az
            msg.ihensa_az = self.ihensa_az
            msg.enc_before_az = self.enc_before_az
            msg.rate_el = self.rate_el
            msg.p_el = self.p_el
            msg.i_el = self.i_el
            msg.d_el = self.d_el
            msg.target_arcsec_el = self.target_arcsec_el
            msg.enc_arcsec_el = self.enc_arcsec_el
            msg.pre_hensa_el = self.pre_hensa_el
            msg.ihensa_el = self.ihensa_el
            msg.enc_before_el = self.enc_before_el
            msg.t_now = self.t_now
            msg.t_past = self.t_past
            pub.publish(msg)
            time.sleep(0.001)
            continue


if __name__ == "__main__":
    rospy.init_node(node_name)
    ant = antenna_move()
    ant.start_thread()
    print("[ROS_antenna_move.py] : START SUBSCRIBE")
    rospy.Subscriber("list_azel", List_coord_msg, ant.set_parameter, queue_size=1000)
    rospy.Subscriber("move_stop", Bool_necst, ant.stop_move)
    # rospy.Subscriber('move_flag', Bool_necst, ant.move_check)
    rospy.Subscriber("emergency_stop", Bool_necst, ant.emergency)
    rospy.Subscriber(
        "status_encoder", Status_encoder_msg, ant.set_enc_parameter, queue_size=1
    )
    rospy.Subscriber("tracking_check", Bool_necst, ant.set_tracking, queue_size=1)
    rospy.spin()
