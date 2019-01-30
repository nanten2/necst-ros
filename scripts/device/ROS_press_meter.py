import rospy
from std_msgs.msg import Float64
import sys
sys.path.append('/home/pi/git/ads')
import ads1256
import time

rospy.init_node('press_raspi')

pub = rospy.Publisher('press_raspi', Float64, queue_size = 1)

def read_press():
    data = ads1256.ads1256_adc()
    data_v = data['CH2']
    """#for recording voltage from press meter
    f = open('voltage20180110.txt', 'a')
    str = '{} {}\n'.format(time.time(), data_v)
    f.write(str)
    f.close()
    """
    #press = (data_v*10/4096-5.)/3.8*1013.25#original
    #press = (239.04*data_v-85.005)/0.75006157584566# conversion voltage into press[hPa]~2019/01
    press = (196.84*data_v + 5.2934)/0.75006157584566# conversion voltage into press[hPa]2019/01/30~
    print(press,"[hPa]", data_v)
    return press

while  1:
    press = read_press()
    pub.publish(press)
    time.sleep(1)
    
