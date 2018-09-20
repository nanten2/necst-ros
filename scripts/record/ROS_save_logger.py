#!/usr/bin/env python3
import rospy
import datetime
import sys, os
from rosgraph_msgs.msg import Log

rospy.init_node('NECST_logger')

###config
save_to = '/home/amigos/log'
try:
    file_name = sys.argv[1]
except:
    file_name = ''
###

def save_file_conf():
    today = datetime.date.today()
    year = today.year
    month = today.month
    day = today.day
    _save_to = os.path.join(save_to, str(year), str(month), str(day))
    _file_name = file_name.split('.')[0]+'.txt'
    if os.path.exists(_save_to):pass
    else:
        os.makedirs(_save_to, exist_ok = True)
        print('Log is save to {}'.format(_save_to))
    return _save_to, _file_name

def save(req):
    ret = save_file_conf()
    savefile = os.path.join(ret[0], ret[1])
    if not req.file == file_name:return
    if '#' in list(req.msg):
        args = req.msg.split('#')[1]
        f_name = req.msg.split('#')[0]
        log = '[{}] : ({}) : {}{}'.format(datetime.datetime.fromtimestamp(req.header.stamp.to_time()), req.file,f_name, args)
        print(log)
    else:
        log = '[{}] : ({}) : {}'.format(datetime.datetime.fromtimestamp(req.header.stamp.to_time()), req.file,req.msg)
        print(log)
    
    f = open(savefile,'a')
    f.write(log+'\n')
    f.close()    


if __name__ == '__main__':
    if file_name == '':
        file_name = input('Pleae input script name [ex : ROS_controller.py]')
    sub = rospy.Subscriber('rosout_agg', Log, save, queue_size=100)
    print('*** Logger Start {} ***'.format(file_name))
    rospy.spin()
