import rospy
import time
from necst.msg import hosei_msg
import n2lites as n2lite

hosei_param = 0
last_update = 0

def save_to_DB():
    global n
    n = n2lite.N2lite("./test.db")
    hosei_param = {"last_update":"float",
                   "daz"        :"float",
                   "de"         :"float",
                   "kai_az"     :"float",
                   "omega_az"   :"float",
                   "eps"        :"float",
                   "kai2_az"    :"float",
                   "omega2_az"  :"float",
                   "kai_el"     :"float",
                   "omega_el"   :"float",
                   "kai2_el"    :"float",
                   "omega2_el"  :"float",
                   "g1"         :"float",
                   "g2"         :"float",
                   "g3"         :"float",
                   "g4"         :"float",
                   "d_el"       :"float",
                   "de_radio"   :"float",
                   "del_radio"  :"float",
                   "cor_v"      :"float",
                   "cor_p"      :"float",
                   "g1_radio"   :"float",
                   "g2_radio"   :"float",
                   "g3_radio"   :"float",
                   "g4_radio"   :"float"
    }
    n.make_table("hosei_parameter", hosei_param)

def callback(req):
    global hosei_param
    global last_update
    print(last_update)
    hosei_param = req
    print("sub")
    hosei_param =  [req.last_update, req.daz, req.de, req.kai_az, req.omega_az, req.eps, req.kai2_az, req.omega2_az, req.kai_el, req.omega_el, req.kai2_el, req.omega2_el, req.g1, req.g2, req.g3, req.g4, req.d_el, req.de_radio, req.del_radio, req.cor_v, req.cor_p, req.g1_radio, req.g2_radio, req.g3_radio, req.g4_radio]
    if not last_update == req.last_update:
        save(hosei_param)
        last_update = req.last_update


def save(hosei_param):
    n.write("hosei_parameter", "", hosei_param, auto_commit = True)
    
if __name__ == "__main__":
    rospy.init_node(__file__[:-3])
    rospy.Subscriber("hosei_publish", hosei_msg, callback, queue_size=1)
    save_to_DB()
    rospy.spin()
