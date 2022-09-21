#!/usr/bin/env python
"""
create: R.Yamada 2020/03/16
"""
import rospy
import ac240client
import os
import sys
sys.path.append('/home/amigos/ros/src/NASCORX_XFFTS')

from std_msgs.msg import Float32
from nascorx_xffts.msg import XFFTS_spec_msg
from nascorx_xffts.msg import XFFTS_pm_msg
from nascorx_xffts.msg import XFFTS_temp_msg
import signal

rospy.init_node('ac240')

signal.signal(signal.SIGINT, signal.SIG_DFL)

pub1 = rospy.Publisher('ac240_spectra_data',XFFTS_spec_msg, queue_size=100)
pub2 = rospy.Publisher('ac240_tp_data',XFFTS_pm_msg, queue_size=100)

def ac240_spec_publisher(d):
    pub1.publish(spec = d['spectrum'],timestamp = d['timestamp'])

def ac240_tp_publisher(d):
    pub2.publish(spec = d['total_power'],timestamp = d['timestamp'])

ac240client.connect(host='172.20.0.41', port=24000, callback=ac240_spec_publisher)
ac240client.connect(host='172.20.0.41', port=24000, callback=ac240_tp_publisher)

signal.signal(signal.SIGINT, signal.SIG_DFL)
