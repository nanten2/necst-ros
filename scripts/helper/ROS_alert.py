#!/usr/bin/env python

import time
import threading
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from necst.msg import Status_weather_msg
from necst.msg import Status_dome_msg
from necst.msg import Dome_msg


class alert(object):

    state = ''
    wind_speed = 0
    rain = 0
    out_humi = 0
    memb = ''
    dome_r = ''
    dome_l = ''
    
    
    def __init__(self):
        self.pub_alert = rospy.Publisher("alert", String, queue_size=10, latch=True)
        self.pub_emergency = rospy.Publisher('emergency_stop', Bool, queue_size = 10, latch = True)
        self.pub_dome = rospy.Publisher('dome_move', Dome_msg, queue_size = 10, latch = True)
        self.sub = rospy.Subscriber("status_weather", Status_weather_msg, self.callback_weather)
        self.sub = rospy.Subscriber("status_dome", Status_dome_msg, self.callback_dome)

    def callback_weather(self, req):
        """ callback : status_weather """
        self.wind_speed = req.wind_sp
        self.rain = req.rain
        self.out_humi = req.out_humi
        
    def callback_dome(self, req):
        """ callback : status_dome """
        status_box = req.status
        self.memb = status_box[6]
        self.dome_r = status_box[2]
        self.dome_l = status_box[4]
        

    def alert(self):
        """ publish : alert """
        msg = String()
        while not rospy.is_shutdown():
            msg.data = self.state
            self.pub_alert.publish(msg)
            self.state = ''
            time.sleep(0.3)
        return
    
    def thread_start(self):
        """ checking alert """
        self.stop_thread = threading.Event()
        self.thread = threading.Thread(target=self.check_thread,name="alert_check_thread")
        self.thread.start()
        
    def check_thread(self):
        """ checking alert """
        emergency = Bool()
        dome = Dome_msg()
        while not rospy.is_shutdown():
            print(self.state, self.wind_speed, self.rain, self.out_humi, self.memb, self.dome_r, self.dome_l)

            if self.rain > 1.:
                self.state = "Warning : probably rainning...\n"
                rospy.logwarn(self.state)
            else:
                pass
            if self.out_humi > 80:
                self.state = "Warning : out_humi over 80 % \n"
                rospy.logwarn(self.state)
            else:
                pass
            
            if 10. < self.wind_speed < 15.:
                self.state = "Warning : wind_speed over 10.\n"
                rospy.logwarn(self.state)
            elif self.wind_speed > 15:
                self.state = "Emergency : wind_speed over 15.\n"
                rospy.logfatal(self.state)
                emergency.data = True
                self.pub_emergency.publish(emergency)
                if self.memb.lower() == 'open':
                    dome.name = 'command'
                    dome.value = 'memb_close'
                    self.pub_dome.publish(dome)
                if self.dome_r.lower() == 'open' or self.dome_l.lower() == 'open':
                    dome.name = 'command'
                    dome.value = 'dome_close'
                    self.pub_dome.publish(dome)
            else:
                pass
            time.sleep(0.1)
        return

if __name__ == "__main__":
    rospy.init_node("alert")
    al = alert()
    al.thread_start()
    al.alert()
