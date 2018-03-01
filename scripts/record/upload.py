import sys
import time
import threading
import traceback
import rospy
### n2db module
sys.path.append("/home/amigos/git")
from n2db import n2database
db = n2database.N2db()
db.authorize()

class upload(object):

    def __init__(self):
        self.save_list = []
        self.project_name = ''
        self.table_name = ''
        pass

    def set_project_name(self, pjt_name):
        self.project_name = pjt_name
        print('set project_name as {0}'.format(self.project_name))

    def set_table_name(self, table_name):
        self.table_name = table_name
        print('set_table name as {0}'.format(self.table_name))
        
    def start_upload(self):
        self.record_thread = threading.Thread(target=self.list_count)
        self.record_thread.setDaemon(True)
        self.record_thread.start()
        return
    
    def list_count(self, count=30):
        while not rospy.is_shutdown():
            if len(save_list) > count: 
                #print('{0}/{1}'.format(len(save_list), count))
                save_list = self.save_list
                self.record(save_list)
                self.save_list = []
                pass
            else:
                #print('{0}/30'.format(len(save_list), count))
                time.sleep(0.1)
                continue
    '''        
    def stop(self):
        self.record_thread.set()
        return
    '''
    def record(self, save_list):
        db.authorize()
        start = time.time()#upload start time
        print('start uploading...')
        if self.project_name == '':
            raise Exception ('set project name')
        if self.table_name == '':
            raise Exception ('set table name')
        try:
            db.INSERT(pjt=self.project_name, table=self.table_name, data=save_list)
            _upload = True
            time.sleep(1)
        except:
            traceback.print_exc()
            #db.authorize()
            _upload = False
            pass
        end = time.time()#upload end time
        if _upload == True:
            rospy.loginfo('upload ended {0} list'.format(len(save_list)))
        else:
            rospy.logerr('failed to upload {0} list'.format(len(save_list)))
        print('{:.2f} sec used\n'.format(end - start))
        

