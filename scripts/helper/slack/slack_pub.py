import rospy
import time
import datetime
import slackweb
from necst.msg import Status_obs_msg 
from necst.msg import Status_weather_msg

f = open('/home/amigos/slackapi.txt')
uri = f.read().split('\n')
slack_link = uri[0]
img_uri = uri[1]
f.close()

rospy.init_node('slack')
before_active = False

slack = slackweb.Slack(slack_link)
param = {'in_temp':0,
         'out_temp':0,
         'in_humi':0,
         'out_humi':0,
         'wind_sp':0,
         'wind_dir':0,
         'press':0,
         'rain':0,
         'cabin_temp1':0,
         'cabin_temp2':0,
         'dome_temp1':0,
         'dome_temp2':0,
         'gen_temp1':0,
         'gen_temp2':0}

def callback(req):
    global before_active
    print('subscrlibe!')
    now = datetime.datetime.now()
    time_ = now.strftime('%Y/%m/%d %H:%M:%S')
    ###Observation start
    if req.active == True and before_active == False:
        slack.notify(text = '>*observation START*\n[(UTC) {}] \n target : {}\n obs_script : {}\n obs_file : {}'.format(time_, req.target, req.obs_script, req.obs_file))
        print('pub!')
        before_active = True
        ###Initialize or Finalize
        if req.target.split(' ')[0] in ['initialize', 'finalize']:
            weather_nortify()
            #nortify_apex_img()
    ###Observation end
    elif req.active == False and before_active == True:
        slack.notify(text = '>*observation END*\n[(UTC) {}] \n target : {}\n obs_script : {}\n obs_file : {}'.format(time_, req.target, req.obs_script, req.obs_file))
        print('pub!')
        before_active = False
    time.sleep(1)

def weather_nortify():
    defalut = "in_temp : {in_temp:04f} [˚C]\n"\
    "out_temp : {out_temp:04f} [˚C]\n"\
    "in_humi : {in_humi:04f} [%]\n"\
    "out_humi : {out_humi:04f} [%]\n"\
    "wind_sp : {wind_sp:04f} [m/s]\n"\
    "wind_dir : {wind_dir:04f} [deg?]\n"\
    "press : {press:04f} [hPa]\n"\
    "rain : {rain} [??]\n"\
    "cabin_temp1 : {cabin_temp1:04f} [˚C]\n"\
    "cabin_temp2 : {cabin_temp2:04f} [˚C]\n"\
    "dome_temp1 : {dome_temp1:04f} [˚C]\n"\
    "dome_temp2 : {dome_temp2:04f} [˚C]\n"\
    "gen_temp1 : {gen_temp1:04f} [˚C]\n"\
    "gen_temp2 : {gen_temp2:04f} [˚C]\n"
    
    attachments = []
    attachment =         {
            "color": "#94D3FD",
            "title": "Weather Status",
            "text":defalut.format(**param)
        }
    attachments.append(attachment)
    slack.notify(attachments=attachments)

def update_weather_status(req):
    global param
    param["in_temp"] = req.in_temp - 273.15
    param["out_temp"]= req.out_temp - 273.15
    param["in_humi"]= req.in_humi
    param["out_humi"]= req.out_humi
    param["wind_sp"]= req.wind_sp
    param["wind_dir"]= req.wind_dir
    param["press"]= req.press
    param["rain"]= req.rain
    param["cabin_temp1"]= req.cabin_temp1 - 273.15
    param["cabin_temp2"]= req.cabin_temp2 - 273.15
    param["dome_temp1"]= req.dome_temp1 - 273.15
    param["dome_temp2"]= req.dome_temp2 - 273.15
    param["gen_temp1"]= req.gen_temp1 - 273.15
    param["gen_temp2"]= req.gen_temp2 - 273.15
    time.sleep(10)
    pass

def nortify_apex_img():
    import requests
    import json
    post_json = {
    "attachments": [{
        "fields": [
            {
                "title": "Apex Weather Status",
                "color": "#81f781"
            }],
        "image_url": "{}".format(img_uri)
        }]}
    requests.post(slack_link, data=json.dumps(post_json))


if __name__ == "__main__":
    ###subscriber
    rospy.Subscriber('/obs_status', Status_obs_msg, callback, queue_size= 1)
    rospy.Subscriber('/status_weather', Status_weather_msg, update_weather_status, queue_size=1)
    
    ###loop
    rospy.spin()
