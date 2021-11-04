import time
import sys
import abc
import logger
from abc import ABCMeta, abstractmethod

sys.path.append("/home/amigos/ros/src/necst/lib")
sys.path.append("/home/amigos/ros/src/necst/scripts/controller")

import ROS_controller
import doppler_nanten

class ObsBase(object, metaclass=abc.ABCMeta):
    @abc.abstractmethod
    def loadobsfile(self):
        pass

    @staticmethod
    @abs.abstractmethod
    def init_ROS_controller(self):
    '''
    微妙にファイルによって中身が違うので、要確認
    '''
        dp = doppler_nanten.doppler_nanten()
        con = ROS_controller.controller()
        con.pub_loggerflag(savedir)  # logger start
        con.dome_track()
        con.move_stop()
        con.pub_encdb_flag(True, os.path.join(savedir, "enc.dat"))

    @abs.abstractmethod
    def fileconfig(self):
        pass

    @staticmethod
    @abs.abstractmethod
    def Handler(self):
    '''
    微妙にファイルによって中身が違うので、要確認
    '''
        con.move_stop()
        con.dome_stop()
        print("!!ctrl + c!!")
        print("Stop antenna")
        con.obs_status(active=False)
        time.sleep(2.0)
        sys.exit()