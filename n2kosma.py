#! /usr/bin/env python3
# -*- coding: utf-8 -*-

#ROS version
import sys
sys.path.append('/home/amigos/ros/src/necst/scripts/controller')
sys.path.append('/home/amigos/ros/src/necst/scripts/record')
import ROS_controller
import ROS_status
con = ROS_controller.controller()
st = ROS_status.read_status()
import threading
import signal
import time
import os
import argparse
#import obs_log
import re
from astropy.coordinates import SkyCoord
import socket
import logging


def emit_colored_ansi(fn):
    def new(*args):
        levelno = args[1].levelno
        if(levelno >= 50):
            color = '\x1b[31m'  # red
        elif(levelno >= 40):
            color = '\x1b[31m'  # red
        elif(levelno >= 30):
            color = '\x1b[33m'  # yellow
        elif(levelno >= 20):
            color = '\x1b[32m'  # green
        elif(levelno >= 10):
            color = '\x1b[35m'  # pink
        else:
            color = '\x1b[0m'  # normal
        args[1].levelname = color + args[1].levelname + '\x1b[0m'  # normal
        return fn(*args)
    return new


def SetupLogging(log_level="info",logging_dir="/home/amigos/NECST/script/obslist/kosma/KOSMA_file_io/logs/",log_name="n2kosma.log"):
   # configure ouput directories
   #
   if not os.path.isdir(logging_dir):
      log.info("{0} doesn't exists, setting dir to '/tmp/'".format(logging_dir))
      logging_dir='/tmp/'
   #
   log_level_dict = {}
   log_level_dict['info'] = logging.INFO
   log_level_dict['warning'] = logging.WARNING   
   log_level_dict['debug'] = logging.DEBUG
   #####################
   # Initialize logger
   #####################
   # to file
   logger_name = os.path.splitext(log_name)[0]   
   log_format = "%(asctime)s: %(levelname)s - %(message)s" # with time
   #log_format = "%(levelname)s: %(message)s" # without time
   log_format = "%(asctime)s: [%(levelname)s: %(filename)s:%(lineno)s - %(funcName)s() ] %(message)s" # with linenumber, functionname message
   logging.basicConfig(format=log_format, level=log_level_dict[log_level])
   log = logging.getLogger(logger_name)
   fh = logging.FileHandler("%s/%s" % (logging_dir,log_name))
   fh.setLevel(log_level_dict[log_level])
   fh.setFormatter(logging.Formatter(log_format))
   log.addHandler(fh)
   # to screen
   ch = logging.StreamHandler()
   log.addHandler(ch)
   print(log_level_dict[log_level])
   fh.setLevel(log_level_dict[log_level])
   f = logging.Formatter(log_format)
   ch.setFormatter(f)
   logging.StreamHandler.emit = emit_colored_ansi(logging.StreamHandler.emit)
   ch.close()
   #
   log.info('writing log to %s/%s' % (logging_dir,log_name))
   return log

def getLogger(log_name="n2kosma.log"):
    return logging.getLogger(os.path.splitext(log_name)[0])
    
    

def initialize():
    import logging
    log = getLogger()
    logging.basicConfig(level='debug')
    
    # Info
    # ----
    log.info('________________________')
    logging.warning('iikagennishitekure')
    name = 'initialize'
    description = 'Initialize antenna'

    # Default param_nameeters
    # ------------------

    opt = ''

    # Argument handler
    # ================

    p = argparse.ArgumentParser(description=description)
    p.add_argument('--opt', type=str,
                   help='For optical. Need 1.')
    p.add_argument('--log_level', type=str,
                   help='logging level debug,info',dest="log_level",
                   default="info"
                   )#defalut=info
    args = p.parse_args()
    # 
    log_level_dict = {}
    log_level_dict['info'] = logging.INFO
    log_level_dict['warning'] = logging.WARNING   
    log_level_dict['debug'] = logging.DEBUG    
    if args.log_level in log_level_dict.keys():
        log.info("setting log level to {0}".format(args.log_level))
        log.setLevel(log_level_dict[args.log_level])
    else:
        log.warning("log level {0} not available".format(args.log_level))
        

    if args.opt is not None: opt = args.opt

    # Main
    # ====
    log.info("starting script {0}".format(name))
    #obs_log.start_script(name)
    log.info("starting weather log")   
    #obs_log.weather_log()
    log.info("starting controller")
    #ctrl = controller.controller()
    con.drive('on')
    log.info("dome_open")
    con.dome_open()

    if opt:
        log.info("memb_open")
        con.memb_open()
    log.info("Init end")
    con.dome_track()
    time.sleep(0.8)
    #con.dome_track_end()

    #obs_log.end_script(name)
    log.info('initialize finished')
    log.info('________________________'+'\n')
    
def handler(num, flame):
    log =getLogger()
    log.info("!!ctrl+C!!")
    log.info("STOP MOVING")
    con.move_stop()
    time.sleep(1)
    con.move_stop()
    con.release_authority()
    time.sleep(2)#to release controller authority
    os.kill(os.getpid(), signal.SIGKILL)
    #sys.exit()
signal.signal(signal.SIGINT, handler)


def ReadKosmaFile(filename):
    log = getLogger("n2kosma.log")
    lines=open(filename, "r").readlines()
    if not os.path.exists(filename):
        log.error("KOSMA file '{0}' not found".format(filename))
    log.debug("reading {0}".format(filename))
    # strip fill path from readwrite_file
    readwrite_file = os.path.basename(filename)
    # create dictionary
    readwrite_dict = {'mod_time' : os.path.getmtime(filename)}
    for i,line in enumerate(lines):
        # check for timestamp in header
        if re.match('.+File update time stamp.+', line):
            timestamp = re.search('.+\s(\d+\.\d+)\s+\S+.+', line).groups()[0]
            readwrite_dict['timestamp']=timestamp             
            continue        
        elif re.match('.+!.+', line):
            result = re.search('(.+)!(.+)', line)
        if result:
            data, description = result.groups()
        else:
            continue
        # if no timestamp found in file, use file timestamp
        if 'timestamp' not in readwrite_dict.keys():
            readwrite_dict['timestamp'] = readwrite_dict['file_timestamp']
        # populate dictionary
        if re.search('(\S+)\s+(\S+)', data) is not None:
            value, variable_found = re.search('(\S+)\s+(\S+)', data).groups()
        elif re.search('\s+(\S+)', data) is not None:        
            variable_found, = re.search('\s+(\S+)', data).groups()
            log.info("{0} variable found with no value ".format(variable_found))
            value="None "
        # parse type
        try:
            if type(value)==int:
                continue
            if "." in value:
                value=float(value)
            else:
                value=int(value)
        except:
            value=value.strip()
        #
        readwrite_dict[variable_found]=value
    #
    return readwrite_dict


class chopper(object):
    set_dir = '/home/amigos/NECST/script/obslist/kosma/KOSMA_file_io/ReadWrite/'
    set_dir = '/home/amigos/ros/src/necst/kosma_setfile/'#for ROS simulator
    def __init__(self):
        self.log = getLogger()
        self.hostname = socket.gethostname()
        self.pid = os.getpid()
        self.server_name = "{0}_{1}".format(__file__,self.__class__.__name__)
        # read kosma file io on start
        self.read_obs2chop()
        self.read_obs2chopsignal()
        #
        self.chopper_server_start()
    
    def read_obs2chopsignal(self, filename='KOSMA_obs2chpsignal.set'):
        self.obs2chopsignal = ReadKosmaFile(self.set_dir+filename)
                
    def read_obs2chop(self, filename='KOSMA_obs2chop.set'):
        #:
        self.obs2chop = ReadKosmaFile(self.set_dir+filename)
        
    def write_chop2obs(self, filename='KOSMA_chop2obs.set'):
        #self.chop2obs = ReadKosmaFile(self.set_dir+filename)        
        #   
        fmt = '%d-%b-%Y  %H:%I:%S'
        current = time.localtime()
       
        chop2obs_reply = {}
        chop2obs_reply['chop_return_cookie'] = self.obs2chopsignal['obs_chop_signal_cookie']
        chop2obs_reply['timestamp'] = time.time()
        chop2obs_reply['timestring'] = date = time.strftime(fmt,current)
        chop2obs_template='''
{0[timestring]}  {0[timestamp]}   File update time stamp   ! {1.server_name} ({1.hostname}:{1.pid})
Y   chop_ok   ! chopper is presently performing ok [Y/N]
     {0[chop_return_cookie]}   chop_return_cookie   ! return cookie to identify proper observation
           0   chop_error   ! flags chopper command syntax/consistency and functionality error (=0: ok)
NONE   chop_error_string   ! Error message not associated with chopper command (=NONE: OK)
           5   chop_error_level   ! Error level for message in chop_error_string [1: trace, 2: debug, 4: info, 5: message, 7: warn, 8: error, 9: fatal]
N   chop_failed   ! chopper failed sometime since last commanded [Y/N]
        '''
        # write file
        tmp_file = self.set_dir+filename+"tmp"
        f = open(tmp_file,'w')
        f.write(chop2obs_template.format(chop2obs_reply,self))
        f.close()
        time.sleep(self.obs2chop["obs_chop_info_update_time"])
        os.rename(tmp_file, self.set_dir+filename)
        #string += date + '  ' + str(c_time) + '   '+ 'File update time stamp'+'   ' +'! load_server (smartXFFTS:2996)' '\n'
        #    
        return
      
    def chopper_server_start(self):    
        while True:
            #self.read_obs2chop()
            #
            if self.obs2chopsignal['mod_time'] != os.path.getmtime(self.set_dir+"KOSMA_obs2chpsignal.set"):
                 #
                 self.log.info("KOSMA_obs2chpsignal.set changed, sending return cookie")
                 self.read_obs2chopsignal()
                 self.write_chop2obs()
            time.sleep(self.obs2chop["obs_chop_info_update_time"])
            
            
        
class mirror(object):
    Pre_timestamp = 0
    set_dir = '/home/amigos/NECST/script/obslist/kosma/KOSMA_file_io/ReadWrite/'
    set_dir = '/home/amigos/ros/src/necst/kosma_setfile/'
    
    def __init__(self):
        self.log = getLogger()
        self.hostname = socket.gethostname()
        self.pid = os.getpid()
        self.server_name = "{0}_{1}".format(__file__ ,self.__class__.__name__)
        self.M4_hot_move()
    
    def Read_set_file_mir(self,file_name = 'KOSMA_obs2load.set'):
        try:
            with open(self.set_dir + file_name, 'r') as data_items_1:
                data_items_1 = data_items_1.read().split('\n')
        except Exception as comment_1:
            self.log.info(comment_1)
            return [-1,None,None]
        data_1 = {}
        for _item in data_items_1:
            _item = _item.strip()
            if len(_item) == 0:
                break
            _item = _item.split('   ')
            _value = _item[0]
            _key = _item[1]
            data_1[_key] = _value
        time_stamp = data_1['File update time stamp']
        time_stamp_ret = time_stamp.split('  ')
        time_stamp = float(time_stamp_ret[2])

        newdata = time_stamp - float(self.Pre_timestamp)
        return [data_1, time_stamp, newdata]

    def Write_set_file_mir(self,file_name = 'KOSMA_load2obs.set'):
        name_list = ['mir_retcookie','mir_status']

        data = {
            'mir_retcookie':(self.mir_retcookie,'! Return value of Load Cookie')
            ,'mir_status':(self.mir_status,'! Load status (Okay=1)')
            }
        string = ''
        fmt = '%d-%b-%Y  %H:%I:%S'
        current = time.localtime()
        c_time = time.time()
        date = time.strftime(fmt,current)
        #string += date + '  ' + str(c_time) + '   '+ 'File update time stamp'+'   ' +'! load_server (smartXFFTS:2996)' '\n'
        string+="{0}  {1}  File update time stamp   !{2.server_name} ({2.hostname}:{2.pid})\n".format(date,c_time,self)
        
        for i in range(len(name_list)):
            name = str(name_list[i])
            value = str(data[name][0])
            comment = str(data[name][1])
            s = '   '
            if len(value) < 12:
                space = 12 - len(value)
                string += ' '*space+value+s+name+s+comment +'\n'
            elif len(value) > 20:
                string += ' '*space+value+s+name+s+comment +'\n'
            else:
                string += '   '+value+s+name+s+comment +'\n'

        # move file instead of direct writing.. minimize down time
        tmp_file = self.set_dir + file_name
        kosma_file = open(tmp_file,'w')
        kosma_file.write(string)
        kosma_file.close()
        self.log.debug("writing {0}".format(file_name))        
        os.rename(tmp_file, self.set_dir+file_name)

        
    def M4_hot_move(self):
        ###initial parameter
        self.mir_retcookie = 0
        first = True
        while True:
            ###for mirror part start
            self.mir_status = 1
            self.Write_set_file_mir()

            ###Read set file and get variables
            while True:
                try:
                    Read_value_mir = self.Read_set_file_mir()
                    break
                except Exception as mir_comment:
                    self.log.info((mir_comment, 'KOSMA_obs2load.set'))
                    time.sleep(0.01)
                    continue
            mir_value = Read_value_mir[0]
            
            # to read set_file completely
            if mir_value == -1:
                time.sleep(0.1)#0.1?
                continue
            try:
                check = mir_value['mir_cookie']
            except:
                continue
            
            self.Pre_timestamp = Read_value_mir[1]
            newdata = Read_value_mir[2]

            if first:#20180329 to stash first file
                first = False
                continue

            if newdata > 0:
                self.mir_status = 0
                self.Write_set_file_mir()
                time.sleep(0.6)###for check 
                ###
                mir_cookie = mir_value['mir_cookie']
                #self.mir_retcookie = mir_cookie###check!!!0829
                ###
                #status = con.read_status()#1027 changed
                
                """#v1
                status = con_r.read_status()###1027 add
                mir_hotcold = status['Current_M4']
                mir_skyload = status['Current_Hot']
                """
                status = st.read_status()
                mir_hotcold = status[6]['position']
                mir_skyload = status[4]['position']
            
                #log.info('[M4] ',mir_hotcold,'[Hot] ',mir_skyload +'\n')
                self.log.info('[M4] {0} [Hot] {1} '.format(mir_hotcold,mir_skyload))
        
                mir_hotcold = int(mir_value['mir_hotcold'])
                mir_skyload = int(mir_value['mir_skyload'])

                if mir_skyload == 1:##load
                    if mir_hotcold == 1:##cold
                        self.log.info('set M4 for cold load')
                        self.log.info('set hotload removed')
                        con.move_m4('in')#MOVE_M4
                        con.move_hot('out')##MOVE_HOT
                        n = 0
                        while True:
                            M4_status = st.read_status()[6]['position']###1027
                            if M4_status.lower() == 'nagoya':### c.f.GitHub module M4.py
                                break
                            else:
                                self.log.info('wait M4...')
                                time.sleep(0.5)
                                n += 1
                                if n > 10:
                                    con.move_m4('in')
                                    n = 0
                                    continue
                        n1 = 0
                        while True:
                            Hot_status = st.read_status()[4]['position']###1027
                            if Hot_status.lower() == 'out':### c.f.GitHub module M4.py
                                break
                            else:
                                self.log.info('wait abs...')
                                time.sleep(0.5)
                                n1 += 1
                                if n1 > 10:
                                    con.move_hot('out')
                                    n1 = 0
                                    continue
        #ln261
                    else:##hot
                        self.log.info('set M4 for sky')
                        self.log.info('set hotload inserted')
                        con.move_m4('in')#move m4 out
                        con.move_hot('in')#move hot in
                        n2 = 0
                        while True:###get m4 status
                            M4_status = st.read_status()[6]['position']###1027
                            if M4_status.lower() == 'nagoya':###GitHub module M4.py
                                break
                            else:
                                self.log.info('wait M4...')
                                time.sleep(0.5)
                                n2 += 1
                                if n2 > 10:
                                    con.move_m4('in')
                                    n2 = 0
                                    continue
                        n3 = 0
                        while True:###get abs status
                            Hot_status = st.read_status()[4]['position']###1027
                            if Hot_status.lower() == 'in':
                                break
                            else:
                                self.log.info('wait abs...')
                                time.sleep(0.5)
                                n3 += 1
                                if n3 > 10:
                                    con.move_hot('in')
                                    n3 = 0
                                    continue
                else:
                    self.log.info('set M4 for sky')
                    self.log.info('set hotload removed')
                    con.move_m4('out')#MOVE_M4
                    con.move_hot('out')##MOVE_HOT
                    n4 = 0
                    while True:
                        M4_status = st.read_status()[6]['position']###1027
                        if M4_status.lower() == 'smart':###GitHub module M4.py
                            break
                        else:
                            self.log.info('wait M4...')
                            time.sleep(0.5)
                            n4 += 1
                            if n4 > 10:
                                con.move_hot('out')
                                n4 = 0
                                continue
                    n5 = 0
                    while True:
                        Hot_status = st.read_status()[4]['position']###1027
                        if Hot_status.lower() == 'out':###GitHub module M4.py
                            break
                        else:
                            self.log.info('wait abs...')
                            time.sleep(0.5)
                            n5 += 1
                            if n5 > 10:
                                con.move_hot('out')
                                n5 = 0
                                continue
                            
                self.mir_status = 1
                self.mir_retcookie = mir_cookie###check!!!0829
                self.Write_set_file_mir()
                status = st.read_status()
                mir_hotcold = status[6]['position']
                mir_skyload = status[4]['position']
                #log.info('ID(mir_cookie)',mir_cookie, ':[M4]',mir_hotcold,' [Hot]',mir_skyload,' [Status]',self.mir_status,' '+'\n')
                self.log.info('ID(mir_cookie) {0} [M4] {1} [Hot] {2} [Status] {3}'.format(mir_cookie,mir_hotcold,mir_skyload,self.mir_status))
            else:
                pass
            time.sleep(0.4)
            continue
            ###for  mirror part end

class telescope(object):
    ###initial parameters
    tel_on_track = 'N'
    pre_otf_position_flag = 0
    update_cycle_time = 1
    Pre_timestamp_tel = 0
    tel_pos_in_range = 'Y'
    tel_return_cookie = 0
    tel_supports_ephemeris = 'Y'
    set_dir = '/home/amigos/NECST/script/obslist/kosma/KOSMA_file_io/ReadWrite/'
    set_dir = '/home/amigos/ros/src/necst/kosma_setfile/'
    
    def __init__(self):
        ###start telescope part
        self.log = getLogger()
        self.hostname = socket.gethostname()
        self.pid = os.getpid()    
        self.server_name = "{0}_{1}".format(__file__,self.__class__.__name__)         
        thread_status = threading.Thread(target = self.write_status)
        thread_status.start()
        self.Kosma_main()
        

    def Read_set_file_tel(self, file_name = 'KOSMA_obs2tel.set'):
        try:
            with open(self.set_dir + file_name, 'r') as data_items_1:
                data_items_1 = data_items_1.read().split('\n')
        except Exception as comment:
            self.log.info(comment)
            return [-1,-1,-1]
        data_1 = {}
        for _item in data_items_1:
            _item = _item.strip()
            if len(_item) == 0:
                break
            _item = _item.split('   ')
            _value = _item[0]
            _key = _item[1]
            data_1[_key] = _value
        time_stamp = data_1['File update time stamp']
        time_stamp_ret = time_stamp.split('  ')
        time_stamp = float(time_stamp_ret[2])

        newdata = time_stamp - float(self.Pre_timestamp_tel)
        return [data_1, time_stamp, newdata]
    
            
    def Write_set_file_tel(self,data1,file_name = 'KOSMA_tel2obs.set'):
        name_list = ['tel_telescope','tel_on_track','tel_lost_track'
         ,'tel_pos_in_range','tel_error','tel_return_cookie'
         ,'tel_plate_scale','tel_latitude','tel_longitude'
         ,'tel_altitude','tel_angle_focal_plane','tel_los_act'
         ,'tel_time_act','tel_time_del','tel_azm_cmd'
         ,'tel_elv_cmd','tel_azm_act','tel_elv_act','tel_supports_ephemeris'
         ,'tel_error_string','tel_error_level'
         ]

        string = ''
        fmt = '%d-%b-%Y  %H:%I:%S'
        current = time.localtime()
        c_time = time.time()
        date = time.strftime(fmt,current)
        #string += date + '  ' + str(c_time) + '   '+ 'File update time stamp'+'   '+'!' + '\n'
        string+="{0}  {1}  File update time stamp   !{2.server_name} ({2.hostname}:{2.pid})\n".format(date,c_time,self)        

        for i in range(len(name_list)):
            name = str(name_list[i])
            value = str(data1[name][0])
            comment = str(data1[name][1])
            s = '   '
            if value == 'NANTEN2':
                string += ' '*25+value+s+name+s+comment+'\n'
            elif len(value) < 12 and value != 'N' and value != 'Y':
                space = 12 - len(value)
                string += ' '*space+value+s+name+s+comment +'\n'
            elif len(value) > 20:
                string += ' '*space+value+s+name+s+comment +'\n'
            elif value == 'N' or value == 'Y':
                string += value+s+name+s+comment +'\n'
            else:
                string += s+value+s+name+s+comment +'\n'
        # write file and then move
        tmp_file = self.set_dir + file_name + "tmp"
        kosma_file = open(tmp_file,'w')
        kosma_file.write(string)
        kosma_file.close()
        # move file instead of direct writing.. minimize down time
        self.log.debug("writing {0}".format(file_name))
        os.rename(tmp_file, self.set_dir+file_name)
        


    def move_otf_mode(self):
        self.tel_on_track = 'N'
        self.tel_return_cookie = self.tel_value['obs_cookie'] 
        self.tel_pos_in_range = 'Y'
        self.tel_supports_ephemeris = 'Y'

        ###parameter        
        lam_on = float(self.tel_value['obs_lam_on'])#[degree]
        bet_on = float(self.tel_value['obs_bet_on'])#[degree]
        
        lam_vel = float(self.tel_value['obs_lam_vel'])#[arcsec/sec]
        bet_vel = float(self.tel_value['obs_bet_vel'])#[arcsec/sec]
        
        coord_sys_on = self.tel_value['obs_coord_sys_on']
        coord_sys_del = self.tel_value['obs_coord_sys_del']
        coord_sys_off = self.tel_value['obs_coord_sys_off']
        
        lam_del = float(self.tel_value['obs_lam_del'])
        bet_del = float(self.tel_value['obs_bet_del'])
        

        lam_off = float(self.tel_value['obs_lam_off'])#[arcsec]
        lam_off = lam_off/3600.
        bet_off = float(self.tel_value['obs_bet_off'])#[arcsec]
        bet_off = bet_off/3600.

        lamda = float(self.tel_value['obs_wavelength'])

        true_angle_del = self.tel_value['obs_true_angle_del']#dcos apply or not

        duration = float(self.tel_value['obs_track_duration'])
        if duration != 0:
            self.log.info('duration != 0')

        #self.log.info('true_angle_del',true_angle_del)
        self.log.info('true_angle_del {0}'.format(true_angle_del))
        if true_angle_del == 'Y':
            dcos_kosma = 1
        else:
            dcos_kosma = 0

        ###start_time check
        current_time = time.time()
        obs_start_time = float(self.tel_value['obs_start_time'])
        start_time = current_time - obs_start_time
        self.log.info('start_time {0}'.format(start_time))

        if start_time <= 0:
            self.log.info('start_time is backward')
            #return
        elif start_time >= 10000000:
            self.log.info('start_time is over 10000000 sec')
            #return
            
        ###for start position(arg for con.otf_scan)
        rampt = 3
        start_x = -lam_vel*rampt-lam_vel/2.#[arcsec]
        start_y = -bet_vel*rampt-bet_vel/2.#[arcsec]


        #self.log.info(start_x,start_y,'start_x','start_y',"###for check")#D
        self.log.info("{0} {1} start_x, start_y ###for check".format(start_x,start_y))#D        
        
        ### To pre_otf position move_telescope
        on_coord = coord_sys_on.lower()
        if duration == 0:
            
            ###stop tracking
            #con.tracking_end()#v1
            con.move_stop()#v2
            if coord_sys_on.lower() == 'j2000' or coord_sys_on.lower() == 'b1950':
                coord_sys_on = 'EQUATORIAL'
            elif coord_sys_on.lower() == 'galactic':
                coord_sys_on = 'GALACTIC'
            elif coord_sys_on.lower() == 'horizon':
                coord_sys_on = 'HORIZONTAL'

            self.log.info('coord_sys_on###'.format(coord_sys_on))
                
        ### 
            if coord_sys_on == 'EQUATORIAL':
                con.onepoint_move(lam_on  ,bet_on , coord = on_coord, off_x = start_x + lam_del, off_y = start_y + bet_del, offcoord = coord_sys_del, dcos = dcos_kosma)
                self.log.info("{0} {1} {2} {3} {4} {5} {6}".format(lam_on,bet_on,on_coord, start_x + lam_del ,start_y+bet_del, coord_sys_del,dcos_kosma))
                #sys.log.info(lam_on, bet_on, on_coord, start_x + lam_del ,start_y+bet_del, coord_sys_del, dcos_kosma)#D
            elif coord_sys_on == 'GALACTIC':
                con.onepoint_move(lam_on ,bet_on ,coord = on_coord, off_x = start_x + lam_del, off_y = start_y + bet_del, offcoord = coord_sys_del, dcos = dcos_kosma)
                self.log.info("{0} {1} {2} {3} {4} {5}".format(lam_on ,bet_on, start_x+lam_del, start_y+bet_del, coord_sys_del,dcos_kosma))                
                #self.log.info(lam_on ,bet_on, start_x+lam_del, start_y+bet_del, coord_sys_del,dcos_kosma)#D
            elif coord_sys_on == 'HORIZONTAL':
                #con.azel_move(lam_on*3600. + lam_del +start_x, bet_on*3600.+bet_del+start_y)
                con.onepoint_move(lam_on + (lam_del+start_x)/3600., bet_on + (bet_del+start_y)/3600.)
                self.log.info("{0} {1}".format(lam_on*3600. + lam_del , bet_on*3600.+bet_del))
                #self.log.info(lam_on*3600. + lam_del , bet_on*3600.+bet_del)
                
            self.log.info('moving...')
            time.sleep(0.001)#分光計の指令値更新を待つ(一応)
            while not st.read_status()[10]:#通り過ぎた場合もTrueになるため
                time.sleep(0.001)
                continue
            time.sleep(0.1)
            while not st.read_status()[10]:#2回目
                time.sleep(0.001)
                continue
            time.sleep(0.1)
            while not st.read_status()[10]:#一応3回目
                time.sleep(0.001)
                continue
            
            self.log.info('Pre_Otf_Position')###-d
            self.tel_on_track = 'Y'
            self.pre_otf_position_flag = 1
            #self.cookie_flag = 1
            
    ###otf scan start(1 line)
        if duration != 0 and self.pre_otf_position_flag == 1:
            self.tel_on_track = 'N'
            on_coord = coord_sys_on.lower()
            if coord_sys_on.lower() == 'j2000' or coord_sys_on.lower() == 'b1950':
                #coord_sys_on = 'EQUATORIAL'
                pass
            elif coord_sys_on.lower() == 'galactic':
                coord_sys_on = 'GALACTIC'
            elif coord_sys_on.lower() == 'horizon':
                coord_sys_on = 'HORIZONTAL'
                lam_on = lam_on*3600 #[arcsec]
                bet_on = bet_on*3600 #[arcsec]
            #start_on = con.otf_scan(lam_on, bet_on, dcos_kosma, coord_sys_on, lam_vel, bet_vel, 1, duration, 3, 0, lamda, 'hosei_230.txt', on_coord,lam_del, bet_del, coord_sys_del)###v1

            ###tmp
            from datetime import datetime
            utc = datetime.utcnow()
            #_list = [utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second, utc.microsecond]
            print(lam_on, bet_on, coord_sys_on, lam_vel, bet_vel, 1, duration, 3, 0, time.time(), lam_del,bet_del, coord_sys_del,dcos_kosma, 'hosei_230.txt',int(lamda))
            start_on = con.otf_scan(lam_on, bet_on, coord_sys_on, lam_vel, bet_vel, 1, duration, 3, 0, time.time(), off_x = lam_del, off_y = bet_del, offcoord = coord_sys_del, dcos = dcos_kosma, hosei = 'hosei_230.txt', lamda = int(lamda))  
            #self.log.info(lam_on, bet_on, dcos_kosma, coord_sys_on, lam_vel, bet_vel, 1, duration, 3, 0, lamda, 'hosei_230.txt', on_coord,lam_del, bet_del,  coord_sys_del)
            to_print = [lam_on, bet_on, dcos_kosma, coord_sys_on, lam_vel, bet_vel, 1, duration, 3, 0, lamda, 'hosei_230.txt', on_coord,lam_del, bet_del,  coord_sys_del]
            self.log.info(([str(ele) for ele in to_print]))
            self.log.info(start_on)
            self.pre_otf_position_flag = 0
    ###for threading (temporary///until controller modified)
            time.sleep(3.5)#lamp
            self.tel_on_track = 'Y'
            time.sleep(duration+2.)
            self.log.info('otf 1 line end')
            self.tel_on_track = 'N'

            ###antenna stop###
            #con.tracking_end()#v1
            con.move_stop()#v2
            
    def move_telescope(self):
        self.pre_otf_position_flag = 0
        self.tel_pos_in_range = 'Y'
        self.tel_on_track = 'N'
        self.tel_supports_ephemeris = 'Y'
        self.tel_return_cookie = self.tel_value['obs_cookie']

        ###stop tracking
        #con.tracking_end()#v1
        con.move_stop()
        
        ###parameter
        coord_on = self.tel_value['obs_coord_sys_on']
        obs_lam_on = float(self.tel_value['obs_lam_on'])#[degree]
        obs_bet_on = float(self.tel_value['obs_bet_on'])#[degree]
        obs_source_name = self.tel_value['obs_source_name']
        obs_wavelength = float(self.tel_value['obs_wavelength'])/1000000.

        lam_off = float(self.tel_value['obs_lam_off'])#[arcsec]
        bet_off = float(self.tel_value['obs_bet_off'])#[arcsec]
        self.log.info('lam_off : {0:3.3f},bet_off {1:3.3f}'.format(lam_off,bet_off))        
        bet_off = bet_off/3600.#[degree]
        lam_off = lam_off/3600.#[degree]
        lam_del = float(self.tel_value['obs_lam_del'])#[arcsec]
        bet_del = float(self.tel_value['obs_bet_del'])#[arcsec]

        coord_map_offsets = self.tel_value['obs_coord_sys_del']
        coord_off = self.tel_value['obs_coord_sys_off']

        ###set lam_off bet_off(off position = lam_on_off, bet_on_off)
        ###coordintate transform

        if coord_on.lower() == 'j2000':
            self.log.info('coord_on = j2000')
            if coord_off.lower() == 'j2000':
                lam_on_off = obs_lam_on + lam_off
                bet_on_off = obs_bet_on + bet_off
                
            if coord_off.lower() == 'b1950':
                self.log.info('coord_off = b1950')
                coord = SkyCoord(obs_lam_on,obs_bet_on,frame = 'fk5',unit = 'deg')
                b1950_ra = coord.fk4.ra.degree + lam_off
                b1950_dec = coord.fk4.dec.degree + bet_off
                ###
                self.log.info(b1950_ra,b1950_dec)
                t_coord = SkyCoord(b1950_ra, b1950_dec, frame = 'fk4',unit = 'deg')
                j2000_ra = t_coord.fk5.ra.degree
                j2000_dec = t_coord.fk5.dec.degree

                lam_on_off = j2000_ra
                bet_on_off = j2000_dec
                self.log.info(type(b1950_ra))###for check -d
            if coord_off.lower() == 'galactic':
                self.log.info('coord_off= {0}'.format(coord_off))
                coord = SkyCoord(obs_lam_on,obs_bet_on,frame = 'fk5',unit = 'deg')
                galac_l = coord.galactic.l.degree + lam_off
                galac_b = coord.galactic.b.degree + bet_off
                self.log.info("{0} {1}".format(galac_l,galac_b))
                ###
                t_coord = SkyCoord(galac_l, galac_b, frame = 'galactic',unit = 'deg')
                j2000_ra = t_coord.fk5.ra.degree
                j2000_dec = t_coord.fk5.dec.degree

                lam_on_off = j2000_ra
                bet_on_off = j2000_dec
            if coord_off.lower() == 'horizon':
                #self.log.info('#####cannot calculate off position#####')
                lam_on_off = obs_lam_on + lam_off
                bet_on_off = obs_bet_on + bet_off
                
        if coord_on.lower() == 'b1950':
            self.log.info('coord_on = b1950')
            if coord_off.lower() == 'j2000':
                self.log.info('coord_off = j2000')
                coord = SkyCoord(obs_lam_on,obs_bet_on,frame = 'fk4',unit = 'deg')
                j2000_ra = coord.fk5.ra.degree + lam_off
                j2000_dec = coord.fk5.dec.degree + bet_off
                self.log.info("{0} {1}".format(j2000_ra,j2000_dec))
                ###
                t_coord = SkyCoord(j2000_ra, j2000_dec,frame = 'fk5',unit = 'deg')
                b1950_ra = t_coord.fk4.ra.degree
                b1950_dec = t_coord.fk4.dec.degree
                
                lam_on_off = b1950_ra
                bet_on_off = b1950_dec
            if coord_off.lower() == 'galactic':
                self.log.info('coord_off= {0}'.format(coord_off))
                coord = SkyCoord(obs_lam_on,obs_bet_on,frame = 'fk4',unit = 'deg')
                galac_l = coord.galactic.l.degree + lam_off
                galac_b = coord.galactic.b.degree + bet_off
                self.log.info("{0} {1}".format(galac_l,galac_b))
                ###
                t_coord = SkyCoord(galac_l, galac_b, frame = 'galactic',unit = 'deg')
                b1950_ra = t_coord.fk4.ra.degree
                b1950_dec = t_coord.fk4.dec.degree
                
                lam_on_off = b1950_ra
                bet_on_off = b1950_dec
            if coord_off.lower() == 'horizon':
                #self.log.info('#####cannot calculate off position#####')
                lam_on_off = obs_lam_on + lam_off
                bet_on_off = obs_bet_on + bet_off
            if coord_off.lower() == 'b1950':
                lam_on_off = obs_lam_on + lam_off
                bet_on_off = obs_bet_on + bet_off

        if coord_on.lower() == 'galactic':
            self.log.info('coord_on = galactic')
            if coord_off.lower() == 'j2000':
                self.log.info('coord_off = j2000')
                coord = SkyCoord(obs_lam_on,obs_bet_on,frame = 'galactic',unit = 'deg')
                j2000_ra = coord.fk5.ra.degree + lam_off
                j2000_dec = coord.fk5.dec.degree + bet_off
                ###
                self.log.info("{0} {1}".format(j2000_ra,j2000_dec))
                t_coord = SkyCoord(j2000_ra,j2000_dec,frame = 'fk5',unit = 'deg')
                galac_l = t_coord.galactic.l.degree
                galac_b = t_coord.galactic.b.degree

                lam_on_off = galac_l
                bet_on_off = galac_b
            if coord_off.lower() == 'b1950':
                self.log.info('coord_off= {0}'.format(coord_off))
                coord = SkyCoord(obs_lam_on,obs_bet_on,frame = 'galactic',unit = 'deg')
                b1950_ra = coord.fk4.ra.degree + lam_off
                b1950_dec = coord.fk4.dec.degree + bet_off
                self.log.info("{0} {1}".format(b1950_ra,b1950_dec))
                ###
                t_coord = SkyCoord(b1950_ra,b1950_dec,frame = 'fk4',unit = 'deg')
                galac_l = t_coord.galactic.l.degree
                galac_b = t_coord.galactic.b.degree
                
                lam_on_off = galac_l
                bet_on_off = galac_b
            if coord_off.lower() == 'horizon':
                #self.log.info('#####cannot calculate off position#####')
                lam_on_off = obs_lam_on + lam_off
                bet_on_off = obs_bet_on + bet_off
            if coord_off.lower() == 'galactic':
                lam_on_off = obs_lam_on + lam_off
                bet_on_off = obs_bet_on + bet_off
                      
        ###planet move telescope
        if obs_source_name[0] == '@' or obs_source_name[0] == '#':
            source_name = obs_source_name
            if source_name == '@SUN' or source_name == '#SUN':
                number = 11
            elif source_name == '@MOON' or source_name == '#MOON':
                number = 10
            elif source_name == '@JUPITER' or source_name == '#JUPITER':
                number = 5
            elif source_name == '@MERCURY' or source_name == '#MERCURY':
                number = 1
            elif source_name == '@MARS' or source_name == '#MARS':
                number = 4
            elif source_name == '@VENUS' or source_name == '#VENUS':
                number = 2
            elif source_name == '@SATURN' or source_name == '#SATURN':
                number = 6
            elif source_name == '@URANUS' or source_name == '#URANUS':###
                number = 7
            elif source_name == '@NEPTUNE' or source_name == '#NEPTUNE':###
                number = 8
            elif source_name == '@PLUTO' or source_name == '#PLUTO':###
                number = 9
            else:
                self.tel_supports_ephemeris = 'N'
                #self.ephemeris_flag = 1
                self.log.info('source_name == {0}'.format(source_name))
                return
            self.log.info('source_name == {0}'.format(source_name))
        
            con.planet_move(int(number), off_x = lam_del, off_y = bet_del, hosei = 'hosei_230.txt', offcoord = coord_map_offsets, lamda=obs_wavelength, az_rate=12000, el_rate=12000, dcos=0)
            to_print = [int(number), 'off_x =', lam_del, 'off_y =', bet_del, 'hosei = ','hosei_230.txt', 'offcoord = ',coord_map_offsets,' lamda=',obs_wavelength, 'az_rate=',12000, 'el_rate=',12000, 'dcos=',0]
            self.log.info(" ".join([str(ele) for ele in to_print]))
            
            self.log.info('moving...')
            time.sleep(0.001)#分光計の指令値更新を待つ(一応)
            while not st.read_status()[10]['tracking']:#通り過ぎた場合もTrueになるため
                time.sleep(0.001)
                continue
            time.sleep(0.1)
            while not st.read_status()[10]['tracking']:#2回目
                time.sleep(0.001)
                continue
            time.sleep(0.1)
            while not st.read_status()[10]['tracking']:#一応3回目
                time.sleep(0.001)
                continue
            self.log.info('tel_on_track = Y')
            self.tel_on_track = 'Y'
            #self.cookie_flag = 1
            return

        if coord_on == 'HORIZON':
            coord_on = 'horizontal'

        if coord_on == 'horizontal':
            if obs_lam_on*3600.+lam_off+lam_del < -240*3600 or obs_lam_on*3600.+lam_off+lam_del >240*3600 or obs_bet_on*3600.+bet_off+bet_del < 0 or obs_bet_on*3600.+bet_off+bet_del > 90*3600:
                self.tel_pos_in_range = 'N'
                self.log.info('!!El range is 0~90[degree]!!')
                return
            #con.azel_move(obs_lam_on*3600.+lam_off+lam_del, obs_bet_on*3600.+bet_off+bet_del)###v1
            con.onepoint_move(obs_lam_on+lam_off/3600.+lam_del/3600, obs_bet_on+bet_off/3600.+bet_del/3600.)
            to_print = ['horizon##',obs_lam_on*3600.+lam_off+lam_del, obs_bet_on*3600.+bet_off+bet_del]
            self.log.info(" ".join([str(ele) for ele in to_print]))
        elif coord_on.lower() == 'galactic':
            con.onepoint_move(lam_on_off, bet_on_off,coord = coord_on, off_x = lam_del, off_y = bet_del, offcoord = coord_map_offsets, lamda = obs_wavelength)
            to_print = [lam_on_off, bet_on_off, 'off_x = ',lam_del, 'off_y = ',bet_del, 'offcoord =', coord_map_offsets,' lamda =', obs_wavelength]
            self.log.info(" ".join([str(ele) for ele in to_print]))
        elif coord_on.lower() == 'j2000' or coord_on.lower() == 'b1950':
            con.onepoint_move(lam_on_off, bet_on_off, coord = coord_on,off_x = lam_del, off_y = bet_del, offcoord = coord_map_offsets, lamda = obs_wavelength)
            # hack from ronan
            self.log.info("{0:3.3f} {1:3.3f} {2} off_x={3} off_y={4} offcoord={5} lamda={6}".format(lam_on_off, bet_on_off, coord_on, lam_del, bet_del,coord_map_offsets,obs_wavelength))
            #print(lam_on_off, bet_on_off, coord_on,'off_x =', lam_del, 'off_y = ',bet_del, 'offcoord = ',coord_map_offsets, 'lamda = ',obs_wavelength)
        self.log.info('moving...')
        time.sleep(0.001)#分光計の指令値更新を待つ(一応)
        while not st.read_status()[10]:#通り過ぎた場合もTrueになるため
            time.sleep(0.001)
            continue
        time.sleep(0.1)
        while not st.read_status()[10]:#2回目
            time.sleep(0.001)
            continue
        time.sleep(0.1)
        while not st.read_status()[10]:#一応3回目
            time.sleep(0.001)
            continue
        print(st.read_status()[10])
        self.log.info('tracking : {0}'.format(st.read_status()[10]))
        self.tel_on_track = 'Y'

    def Kosma_main(self):
        first = True
    ###for telescope control part start
        while True:
            while True:
                try:
                    Read_value_tel = self.Read_set_file_tel()
                    break
                except Exception as comment:
                    self.log.info('{0} : KOSMA_obs2tel.set'.format(comment))
                    time.sleep(0.05)
                    continue
            
            self.tel_value = Read_value_tel[0]
            self.Pre_timestamp_tel = Read_value_tel[1]
            newdata = Read_value_tel[2]
            
            if first:#20180329 to stash first file
                first = False
                continue
            
            ###to read set_file completely
            try:
                check = self.tel_value['obs_chop_beam']
            except:
                continue
            ###
            if newdata > 0:
                otf_mode = self.tel_value['obs_otf_mode']
                self.update_cycle_time = float(self.tel_value['obs_tel_info_update_time'])
                # changed by ronan
                #self.tel_return_cookie = self.tel_value['obs_cookie'] 
                self.log.info('otf_mode  : {0}'.format(otf_mode))
                if otf_mode == 'Y':
                    self.move_otf_mode()
                else:
                    self.move_telescope()
                time.sleep(0.1)## 
            time.sleep(0.1)##
            continue
       
    def write_status(self):
        while True:
            self.log.debug(self.update_cycle_time)
            if self.update_cycle_time != 0.:
                time.sleep(self.update_cycle_time)
            else:
                time.sleep(1)
                continue
            status = st.read_status()
            ###b_time = time.time()
            #print(status)
            command_Az = status[0]['command_az']
            command_El = status[0]['command_el']
            current_Az = status[2]['encoder_az']
            current_El = status[2]['encoder_el']
            telescope_message = "OK" # status message from telescope
            telescope_error_level = 4 # level of messages back from telescope
            #self.tel_pos_in_range = 'Y'
            #self.tel_supports_ephemeris = 'Y' #0905
            data1 = {
                'tel_telescope':('NANTEN2','! Telescope Identifier (for display)')
                ,'tel_on_track':(self.tel_on_track,'! Y if tracking on commanded position/track within tolerance [Y/N]')
                ,'tel_lost_track':('N','! Y if tracking got beyond tolerance since start of track [Y/N]')
                ,'tel_pos_in_range':(self.tel_pos_in_range,'! Y if commanded position is within telescope range [Y/N]')
                ,'tel_error':('0','! flags telescope command syntax/consistency and functional errors (=0: ok)')
                ,'tel_return_cookie':(self.tel_return_cookie,'! return cookie to identify proper observation')
                ,'tel_plate_scale':('1','! plate scale of focal plane [arcsec/mm]')
                ,'tel_latitude':('-22.96995611','! latitude of observatory (needed for Doppler correction) [degree]')
                ,'tel_longitude':('67.70308139','! longitude of observatory (needed for Doppler correction) [degree;+west]')
                ,'tel_altitude':('4863.85','! height above sea level (needed for Doppler correction) [meter]')
                ,'tel_angle_focal_plane':('0','! angle between on_source coord sys and focal plane [degree]')
                ,'tel_los_act':('0','! actual los-angle [degree]')
                ,'tel_time_act':(str(time.time()),'! time when status data were valid [seconds since 1970 UTC]')
                ,'tel_time_del':('0','! time since start of track (may be negative, if start in the future) [seconds]')
                ,'tel_azm_cmd':(command_Az,'! commanded azimuth (for tracking display) [degree;cw toward east]')
                ,'tel_elv_cmd':(command_El,'! commanded elevation (for tracking display and for atmospheric calibration) [degree]')
                ,'tel_azm_act':(current_Az,'! actual azimuth (for tracking display) [degree;cw toward east]')
                ,'tel_elv_act':(current_El,'! actual elevation (for tracking display) [degree]')
                ,'tel_supports_ephemeris':(self.tel_supports_ephemeris,'! Y if telescope can calculate ephemeris, N if telescope expects observing task to do this for non-inertial objects [Y/N]')
                ,'tel_error_string' : (telescope_message,'Error message associated with telescope command (=NONE: OK)')
                ,'tel_error_level' : (telescope_error_level,'Error level for message in tel_error_string [1: trace; 2: debug; 4: info; 5: message; 7: warn; 8: error; 9: fatal]')
                 }
            self.Write_set_file_tel(data1)
            continue


####main part####
#SetupLogging(log_level="info",logging_dir="/home/amigos/NECST/script/obslist/kosma/KOSMA_file_io/logs/",log_name="n2kosma.log")
SetupLogging(log_level="info",logging_dir="/home/amigos/ros/src/necst/log",log_name="n2kosma.log")#for simulator
initialize()
thread_mirror_hot = threading.Thread(target = mirror)
thread_mirror_hot.setDaemon(True)
thread_mirror_hot.start()
thread_chopper = threading.Thread(target = chopper)
thread_chopper.setDaemon(True)
thread_chopper.start()
telescope()
#while True:time.sleep(2)


