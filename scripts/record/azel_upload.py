#example
import upload, time, rospy
from necst.msg import Status_encoder_msg

class encoder_up(object):
    def __init__(self):
        self.save_list = []
        pass

    def make_list(self, req):
        self.save_list.append([req.utc, req.enc_az/3600., req.enc_el/3600.])
        time.sleep(1)
        pass

    def list_up(self):
        while not rospy.is_shutdown():
            print(len(self.save_list))
            if len(self.save_list) > 30:
                save_list_sub = self.save_list
                self.save_list = []
                u.record(save_list_sub)
                continue
            time.sleep(0.1)
            
if __name__ == '__main__':
    rospy.init_node('upload_azel_to_spreadsheet')
    u = upload.upload()
    u.set_project_name('Telescope')
    u.set_table_name('AzEl')
    en = encoder_up()
    sub = rospy.Subscriber('status_encoder', Status_encoder_msg, en.make_list, queue_size = 1)
    en.list_up()
