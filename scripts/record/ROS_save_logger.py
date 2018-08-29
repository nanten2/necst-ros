#!/usr/bin/env python3
import rospy
import datetime
from rosgraph_msgs.msg import Log

rospy.init_node('NECST_logger')


def hoge(req):
    log = '[{}] : ({}) : {} |F:{}|Ln:{}|\n'.format(datetime.datetime.fromtimestamp(req.header.stamp.to_time()), req.file, req.msg,req.function, req.line)
    print(log)
    f = open('hogelog.txt','a')
    f.write(log)
    f.close()
    
sub = rospy.Subscriber('rosout_agg', Log, hoge, queue_size=100)
rospy.spin()
