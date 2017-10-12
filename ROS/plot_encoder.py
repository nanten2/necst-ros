import signal
def handler(num, flame):
    print("!!ctrl+C!!")
    f.close()
    sys.exit()

import matplotlib.pyplot as plt
import numpy as np
####
import rospy
from ros_start.msg import Status_encoder_msg
####
class plot_encoder(object):

    az = 0
    el = 0
    ####
    def func(self, req):
        self.az = req.enc_az
        self.el = req.enc_el
        return
    ####

    def plot(self):
        
        plt.grid()
        for i in range(0, 1000000):
            print("loop : ", i)
            #plt.clf()
            #plt.grid()
            '''
            try:
                file = open("log.txt","r")
                line = file.readlines()
                x = float(line[5].split("\n")[0])
                y = float(line[6].split("\n")[0])
                print(round(x,3), round(y,3))
            except:
                try:
                    file = open("log.txt","r")
                    line = file.readlines()
                    x = float(line[5].split("\n")[0])
                    y = float(line[6].split("\n")[0])
                except:
                    pass
            '''
            x = self.az/3600.
            y = self.el/3600.
            #plt.plot(x, y,"o")
            #plt.xlim(-180., +180.)
            #plt.ylim(0., 90.)
            x1 = x*3600.-800
            y1 = y*3600.-800
            x2 = x*3600.+800
            y2 = y*3600.+800
            plt.xlim(x1, x2)
            plt.ylim(y1, y2)
            plt.plot(x*3600, y*3600,"yellow",alpha = 0.5,marker="*",markersize=12 ,markeredgecolor="b", markeredgewidth=0.7)
            plt.ylabel('elevation[arcsec]')
            plt.xlabel('azimuth[arcsec]')
            plt.title('tk controller plot')
            #plt.savefig("hot_monitor.png")
            plt.pause(0.08)

rospy.init_node("encoder_plot")
en = plot_encoder()
sub = rospy.Subscriber("status_encoder", Status_encoder_msg, en.func)
en.plot()

