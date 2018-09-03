#!/usr/bin/env python3
import rospy
import datetime
import sys, os
from rosgraph_msgs.msg import Log

rospy.init_node('NECST_logger')

save_to = '/home/amigos/log'
def hoge(req):
    if not req.file == 'ROS_controller.py':return
    if '#' in list(req.msg):
        args = req.msg.split('#')[1]
        f_name = req.msg.split('#')[0]
        log = '[{}] : ({}) : {}{}'.format(datetime.datetime.fromtimestamp(req.header.stamp.to_time()), req.file,f_name, args)
        print(log)
    else:
        log = '[{}] : ({}) : {}'.format(datetime.datetime.fromtimestamp(req.header.stamp.to_time()), req.file,req.msg)
        print(log)
    
    f = open(path,'a')
    f.write(log+'\n')
    f.close()    


if __name__ == '__main__':
    sub = rospy.Subscriber('rosout_agg', Log, hoge, queue_size=100)
    try:
        file_name = sys.argv[1]
    except:
        file_name = input('Please input file name : ')
    path = os.path.join(save_to, file_name)
    if os.path.exists(save_to):pass
    else:os.mkdir(save_to)
    print('Log is save to {}'.format(path))
    print('*** Logger Start ***')
    rospy.spin()
