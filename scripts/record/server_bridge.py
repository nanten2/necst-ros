import time
import ast
import threading
from datetime import datetime as dt
import numpy as np
import sys
sys.path.append("/home/amigos/git/")
from n2db import n2database
db = n2database.N2db()
db.authorize()
from necst.msg import NECST_list_msg
from necst.msg import Status_weather_msg
from necst.msg import Read_status_msg
from necst.msg import String_necst
from necst.msg import Status_encoder_msg
import signal
def handler(signal, frame):
     rospy.on_shutdown(_end)

def _end():
     print("rospy down...")
signal.signal(signal.SIGINT, handler)



import rospy
node_name = "bridge"

func_status = {"m2_controller":False, "abs_controller":False,"m4_controller":False, "worldcoordinate_linear":False, "worldcoordinate_planet":False, "worldcoordinate_otf":False, "worldcoordinate_onepoint":False, "encoder_status":False, "azel_list":False, }
node_status = ""
weather_status = {'in_temp': 14.33, 'out_temp': 15.78, 'in_humi': 1.0, 'out_humi': 8.0, 'wind_sp': 5.81, 'wind_dir': 268.0, 'press': 570.105, 'rain': -0.0, 'cabin_temp1': 21.3, 'cabin_temp2': 31.6, 'dome_temp1': 9.8, 'dome_temp2': 12.9, 'gen_temp1': 9.8, 'gen_temp2': 12.9}
encoder_status = {"enc_az":0, "enc_el":45*3600.}


def read():
     global node_status
     while not rospy.is_shutdown():
          ct=dt.utcnow()
          date = ct.strftime("%Y-%m-%d")
          try:
               data1=db.SELECT("Node_status", date, date, num=1)
               if data1[1]:
                    data_dict1 = ast.literal_eval(data1[1])
                    func_status.update(data_dict1)
               if data1[2]:
                    node_status = ast.literal_eval(data1[2])
                    print(node_status)               
          except Exception as e:
               rospy.logerr(e)
          time.sleep(0.01)
          try:
               data2=db.SELECT("Weather", date, date, num=1)
               if data2[1]:
                    data_dict2 = ast.literal_eval(data2[1])
                    weather_status.update(data_dict2)               
          except Exception as e:
               rospy.logerr(e)
          time.sleep(0.01)
          try:
               data3=db.SELECT("AzEl", date, date, num=1)
               if data3[1]:
                    data_dict3 = ast.literal_eval(data3[1])
                    encoder_status.update(data_dict3)               
          except Exception as e:
               rospy.logerr(e)
          time.sleep(0.01)
     return

def node_active():
     fmsg = NECST_list_msg()
     fmsg.from_node=node_name     
     nmsg = String_necst()
     nmsg.from_node=node_name
     while not rospy.is_shutdown():
          ct=dt.utcnow()
          date = ct.strftime("%Y-%m-%d")
          try:
               data1=db.SELECT("Node_status", date, date, num=1)
               if data1[1]:
                    data_dict1 = ast.literal_eval(data1[1])
                    func_status.update(data_dict1)
                    param = func_status
                    fmsg.m2=param["m2_controller"]
                    fmsg.m4=param["m4_controller"]
                    fmsg.hot=param["abs_controller"]
                    fmsg.scan_linear = param["worldcoordinate_linear"]
                    fmsg.scan_onepoint = param["worldcoordinate_onepoint"]
                    fmsg.scan_planet = param["worldcoordinate_planet"]
                    fmsg.scan_otf = param["worldcoordinate_otf"]          
                    fmsg.encoder = param["encoder_status"]
                    fmsg.azel_list = param["azel_list"]
                    fmsg.timestamp=time.time()
                    pub_func.publish(fmsg)
               if data1[2]:
                    node_status = ast.literal_eval(data1[2])
                    param = str(node_status)
                    nmsg.data=param
                    nmsg.timestamp=time.time()
                    pub_node.publish(nmsg)
          except Exception as e:
               rospy.logerr(e)
          time.sleep(0.1)
     return
     
def weather():
     #wmsg = Status_weather_msg()
     wmsg = Read_status_msg()
     wmsg.from_node=node_name
     while not rospy.is_shutdown():
          ct=dt.utcnow()
          date = ct.strftime("%Y-%m-%d")          
          try:
               data2=db.SELECT("Weather", date, date, num=1)
               if data2[1]:
                    data_dict2 = ast.literal_eval(data2[1])
                    weather_status.update(data_dict2)
                    param = weather_status
                    wmsg.InTemp = param["in_temp"]
                    wmsg.OutTemp = param["out_temp"]
                    wmsg.InHumi = param["in_humi"]
                    wmsg.OutHumi = param["out_humi"]
                    wmsg.WindSp = param["wind_sp"]
                    wmsg.WindDir = param["wind_dir"]
                    wmsg.Press = param["press"]
                    wmsg.Rain = param["rain"]
                    wmsg.CabinTemp1 = param["cabin_temp1"]
                    wmsg.CabinTemp2 = param["cabin_temp2"]
                    wmsg.DomeTemp1 = param["dome_temp1"]
                    wmsg.DomeTemp2 = param["dome_temp2"]
                    wmsg.GenTemp1 = param["gen_temp1"]
                    wmsg.GenTemp2 = param["gen_temp2"]
                    wmsg.timestamp = time.time()
                    pub_weather.publish(wmsg)
          except Exception as e:
               rospy.logerr(e)
          time.sleep(0.1)
     return

def encoder():
     emsg = Status_encoder_msg()
     emsg.from_node = node_name
     while not rospy.is_shutdown():
          ct=dt.utcnow()
          date = ct.strftime("%Y-%m-%d")          
          try:
               data3=db.SELECT("AzEl", date, date, num=1)
               if data3[1]:
                    data_dict3 = ast.literal_eval(data3[1])
                    encoder_status.update(data_dict3)
                    param = encoder_status
                    emsg.enc_az = param["enc_az"]
                    emsg.enc_el = param["enc_el"]
                    pub_encoder.publish(emsg)
          except Exception as e:
               rospy.logerr(e)
          time.sleep(0.1)
     return

if __name__ == "__main__":
     rospy.init_node(node_name)
     pub_func = rospy.Publisher("bridge_func", NECST_list_msg, queue_size=1)
     pub_node = rospy.Publisher("bridge_node", String_necst, queue_size=1)     
     pub_weather = rospy.Publisher("bridge_weather", Read_status_msg, queue_size=1)
     pub_encoder = rospy.Publisher("bridge_encoder", Status_encoder_msg, queue_size=1)     
     
     #th = threading.Thread(target=read)
     #th.start()
     #th = threading.Thread(target=function_active)
     #th.start()
     th = threading.Thread(target=node_active)
     th.start()     
     th = threading.Thread(target=weather)
     th.start()
     th = threading.Thread(target=encoder)
     th.start()     
     rospy.spin()
     


     
