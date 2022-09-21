#!/usr/bin/env python3


name  = 'all_logger_tmp' 

import time
import threading
import necstdb
import pathlib

import rospy
import std_msgs.msg
import necst.msg
import davis.msg
import nascorx_xffts.msg

ignore_topics = [
    '/rosout',
    '/rosout_agg',
    '/hosei_parameter',
    '/list_azel',
    '/read_status',
    '/status_antenna',
    '/status_pid',
    '/alert',
    '/wc_list'
]

ignore_types = [
    'unknown type',
]


def get_current_topic_list():
    topic_list = rospy.get_published_topics()
    for topic in topic_list[:]:
        if topic[0] in ignore_topics:
            topic_list.remove(topic)
        elif topic[1] in ignore_types:
            topic_list.remove(topic)
            pass
        continue
    return topic_list

def make_subscriber(topic):
    topic_name = topic[0]
    topic_type = eval(topic[1].replace('/', '.msg.'))
    rospy.Subscriber(
        name = topic_name,
        data_class = topic_type,
        callback = callback,
        callback_args = topic_name,
        queue_size = 1
    )

def callback(req, arg):
    slots = [{'key': key,
              'type': type_,
              'value': req.__getattribute__(key)}
             for key, type_
             in zip(req.__slots__, req._slot_types)]

    data = {'topic': arg, 'received_time': time.time(), 'slots': slots}

    log.regist(data)
    return

class db_logger_operation(object):
    
    def __init__(self):
        self.data_list = []
        self.table_dict = {}
        #self.db_dir = '/media/usbdisk/data/'
        self.db_dir = '/home/amigos/hdd/data/'
        self.db_path = ''
        t1 = time.time()
        self.sub_path = rospy.Subscriber(
            name = '/logger_path',
            data_class = std_msgs.msg.String,
            callback = self.callback_path,
            queue_size = 1,
        )

        self.th = threading.Thread(target= self.loop)
        self.th.start()
        pass

    def regist(self, data):
        if self.db_path != '':
            self.data_list.append(data)
            pass
        return

    def callback_path(self, req):
        if req.data != '':
            self.db_path = ''
            self.data_list = []
            self.close_tables()
            self.db = necstdb.opendb(self.db_dir + req.data, mode = 'w')
            self.db_path = req.data
            print(req.data)
            time.sleep(0.1)

        else:
            self.db_path = req.data
            pass
        return

    def close_tables(self):
        tables = self.table_dict
        self.table_dict = {}
        [tables[name].close() for name in tables]
        return

    
    def loop(self):

        while True:
            if len(self.data_list) ==0:
                self.close_tables()
                if rospy.is_shutdown():
                    break
                time.sleep(0.01)
                continue
            print(len(self.data_list)) 
            d = self.data_list.pop(0)
            print(d['topic'])
            table_name = d['topic'].replace('/', '-').strip('-')
            table_data = [d['received_time']]
            table_info = [{'key': 'received_time',
                           'format': 'd',
                           'size': 8}]
            
            for slot in d['slots']:
                if slot['type'].startswith('bool'):
                    info = {'format': 'b', 'size': 1}
                
                elif slot['type'].startswith('byte[]'):
                    continue

                elif slot['type'].startswith('byte'):
                    info = {'format': '{0}s'.format(len(slot['value'])), 'size': len(slot['value'])}

                elif slot['type'].startswith('char[]'):
                    continue

                elif slot['type'].startswith('char'):
                    info = {'format': 'c', 'size': 1}
                    if isinstance(slot['value'], str):
                        slot['value'] = slot['value'].encode()
                        pass

                elif slot['type'].startswith('float32'):
                    info = {'format': 'f', 'size': 4}

                elif slot['type'].startswith('float64'):
                    info = {'format': 'd', 'size': 8}

                elif slot['type'].startswith('int8'):
                    info = {'format': 'b', 'size': 1}

                elif slot['type'].startswith('int16'):
                    info = {'format': 'h', 'size': 2}

                elif slot['type'].startswith('int32'):
                    info = {'format': 'i', 'size': 4}

                elif slot['type'].startswith('int64'):
                    info = {'format': 'q', 'size': 8}
                
                elif slot['type'].startswith('string[]'):
                    continue

                elif slot['type'].startswith('string'):
                    info = {'format': '{0}s'.format(len(slot['value'])), 'size': len(slot['value'])}
                    if isinstance(slot['value'], str):
                        slot['value'] = slot['value'].encode()
                        pass
                        
                elif slot['type'].startswith('uint8'):
                    info = {'format': 'B', 'size': 1}

                elif slot['type'].startswith('unit16'):
                    info = {'format': 'H', 'size': 2}

                elif slot['type'].startswith('unit32'):
                    info = {'format': 'I', 'size': 4}

                elif slot['type'].startswith('unit64'):
                    info = {'format': 'Q', 'size': 8}
                else:
                    continue

                if isinstance(slot['value'], tuple):
                    # for MultiArray
                    dlen = len(slot['value'])
                    info['format'] = '{0:d}{1:s}'.format(dlen, info['format'])
                    info['size'] *= dlen
                    table_data += slot['value']
                else:
                    table_data += [slot['value']]
                    pass

                info['key'] = slot['key']
                table_info.append(info)

            if table_name not in self.table_dict:
                self.db.create_table(table_name,
                            {'data': table_info,
                             'memo': 'generated by db_logger_operation',
                             'version': necstdb.__version__,})

                self.table_dict[table_name] = self.db.open_table(table_name, mode='ab')
                pass
            self.table_dict[table_name].append(*table_data)
            continue
        return

if __name__ == '__main__':
    rospy.init_node(name)

    log = db_logger_operation() 
    
    subscribing_topic_list = []
    while not rospy.is_shutdown():
        current_topic_list = get_current_topic_list()
        print('topic num: '+str(len(current_topic_list)))
        for topic in current_topic_list:
            if topic not in subscribing_topic_list:
                make_subscriber(topic)
                subscribing_topic_list.append(topic)
                pass
            continue
        time.sleep(1)
        continue

