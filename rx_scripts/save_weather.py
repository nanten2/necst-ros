import rospy, os
import time
import datetime

from necst.msg import Status_weather_msg

##config##
save_dir = '/home/amigos/data/weather/'

rospy.init_node('save_weather_for_rx')

def save(req):
    now = datetime.datetime.now()
    save_file = '{}{}/{}/{}_weather.txt'.format(save_dir, now.year, now.month, now.day)
    if not os.path.exists(save_file):
        os.makedirs('{}{}/{}'.format(save_dir, now.year, now.month), exist_ok=True)
    f = open(save_file, 'a')
    log = '{} {} {}\n'.format(time.time(), req.dome_temp1, req.dome_temp2)
    f.write(log)
    f.close()
    print('done save')
    time.sleep(60)

if __name__ == '__main__':
    sub = rospy.Subscriber('status_weather', Status_weather_msg, save, queue_size = 1)
    rospy.spin()
