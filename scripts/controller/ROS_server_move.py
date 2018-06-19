import os
import sys
import time
import ast
import atexit
from datetime import datetime as dt
import rospy
import rosnode
from necst.msg import Move_mode_msg
from necst.msg import Otf_mode_msg
from necst.msg import Dome_msg
from necst.msg import Read_status_msg
from necst.msg import Achilles_msg
from necst.msg import Bool_necst
from necst.msg import String_necst
from necst.msg import Int64_necst
from necst.msg import Float64_necst
from NASCORX_XFFTS.msg import XFFTS_para_msg
sys.path.append("/home/amigos/ros/src/necst/lib")

def drive(req):
    con.drive(req.data)
    return

def onepoint(req):
    print(req)    
    con.onepoint_move(req.x, req.y, req.coord, req.off_x, req.off_y, req.offcoord, req.hosei, req.lamda, req.dcos, req.limit)
    return

def linear(req):
    return

def planet(req):
    print(req)
    con.planet_move(req.planet, req.off_x, req.off_y, req.offcoord, req.hosei, req.lamda, req.dcos, req.limit)
    print("ok")
    return

def observation(req):
    #os.system("python /home/amigos/ros/src/necst/obs_scripts/ROS_ps.py --obsfile ps_test.obs")
    #os.system("")
    return

def stop(req):
    con.move_stop()
    return

def emergency(req):
    con.move_stop()
    #con.emergency <-- I want to obs_script also ...
    return

def otf(req):
    return

def dome(req):
    print("dome", req)
    if req.data == "tracking":
        con.dome_track()
    elif req.data == "trackend":
        con.dome_track_end()
    else:
        con.dome(req.data)
    return

def domemove(req):
    print("domemove",req)
    print(type(req.data))
    con.dome_move(req.data)
    print("move ok")
    return
    
def memb(req):
    print("memb",req)    
    con.memb(req.data)
    return

def m4(req):
    con.move_m4(req.data)
    return

def hot(req):
    print("hot : ", req.data)
    con.move_hot(req.data)
    return 

def m2(req):
    print("m2 : ", req.data)
    con.move_m2(req.data)
    return

def ac240(req):
    return

def xffts(req):
    return

def authority(req):
    print("authority : ", req.data)
    if req.data == "get":
        con.get_authority()
    elif req.data == "release":
        con.release_authority()
    else:
        pass
    return

if __name__ == "__main__":
    import ROS_controller
    con = ROS_controller.controller()
    rospy.Subscriber("WebDrive", String_necst, drive, queue_size = 1)
    rospy.Subscriber("WebOnepoint", Move_mode_msg, onepoint, queue_size=1, )
    rospy.Subscriber("WebLinear", Move_mode_msg, linear, queue_size=1,)        
    rospy.Subscriber("WebPlanet", Move_mode_msg, planet, queue_size=1,)        
    rospy.Subscriber("WebStop", Bool_necst, stop, queue_size = 1,)
    rospy.Subscriber("WebEmergency", Bool_necst, emergency, queue_size = 1,)    
    rospy.Subscriber("Web_otf", Otf_mode_msg, otf, queue_size = 1,)
    rospy.Subscriber("WebObservation", String_necst, observation, queue_size = 1,)    
    rospy.Subscriber("WebDome", String_necst, dome, queue_size = 1,)
    rospy.Subscriber("WebDomeMove", Float64_necst, domemove, queue_size = 1,)    
    rospy.Subscriber("WebMemb", String_necst, memb, queue_size = 1,)    
    rospy.Subscriber('WebM4', String_necst, m4, queue_size = 1,)
    rospy.Subscriber("WebHot", String_necst, hot, queue_size = 1,)
    rospy.Subscriber("WebM2", Float64_necst, m2, queue_size=1)
    print("ok")
    rospy.Subscriber("WebAchilles", Achilles_msg, ac240, queue_size=1)
    rospy.Subscriber("WebXFFTS", XFFTS_para_msg, xffts, queue_size=1)
    rospy.Subscriber("WebAuthority", String_necst, authority, queue_size=1)
    print("ok")
    rospy.spin()
    print("error")
