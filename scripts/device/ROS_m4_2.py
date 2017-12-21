import os, datetime, linecache
import libMotor as lib
import NASCORX_System.device.CPZ7415V as CPZ7415V
import sys
import rospy
import time
import threading

from necst.msg import Status_m4_msg
from std_msgs.msg import String

class m4(object):
    """
    DESCRIPTION
    ================
    This class controls the M4.

    ARGUMENTS
    ================
    1. board: M4 class is oprated by board of 'CPZ7415Va'
        Type: string
        board: 'CZP7415Va'
    """
    position = ''
    
    def __init__(self):
        dev = 1 # CPZ7415Va
        LIMIT = 130
        axis = lib.MOTOR_ZAXIS
        log_file = M4_log.txt
        self.mtnc = CPZ7415V.cpz7415v(dev=dev)

    def start_thread(self):
        th = threading.Thread(target = self.pub_status)
        th.setDaemon(True)
        th.start()
        return

    

    def _log(self, con='HOT'):
        """        
        DESCRIPTION
        ================
        This function logs the M4 state means UP or DOWN in the logfile.
        
        ARGUMENTS
        ================
        1. con: M4 state
            Type: str
            Default: HOT

        RETURNS
        ================
        Nothing.
        """
        da = ['{0:%Y-%m-%d %H:%M:%S}'.format(datatime.datetime.now()), ' ' , '{0}\n'.format(con)]
        os.chdir('')
        f = open('{0}'.format(log_file), 'a')
        f.writelines(da)
        f.close()
        os.chdir('')
        return

    def _state(self):
        """        
        DESCRIPTION
        ================
        This function logs the M4 state means UP or DOWN in the logfile.
        
        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        Nothing.
        """
        f = open('{0}'.format(log_file), 'r')
        num = sum(1 for line in open(('{0}'.format(log_file))))
        ret = linecache.getline('{0}'.format(log_file), int(num))
        return ret
        
    def move(self, req):
        """
        DESCRIPTION
        ================
        This function moves the M4 to UP.
        
        ARGUMENTS
        ================
        State of the M4 up passes the RFsignal to the NASCORX.

        RETURNS
        ================
        Nothing.
        """
        if req.data == 'UP':
            if self._state.split()[2] == 'DOWN':
                self.mtnc.move_ccw(strk=[LIMIT,1,1,1], axis=axis)
                self._log(con='UP')
                self.close_box()
                return
            else:
                print('!!!!ERROR!!!!\n'
                      'M4 has already moved to UP.')

        if req.data == 'DOWN':
            if self._state.split()[2] = 'UP':
                self.mtnc.move_cw(strk=[LIMIT,1,1,1], axis=axis)
                self._log(con='DOWN')
                self.close_box()
                return
            else:
                print('!!!!ERROR!!!!\n'
                      'M4 has already moved to DOWN.')
                  
            
    def query_position(self):
        """        
        DESCRIPTION
        ================
        This function queries the M4 position.
        (The M4 positino means state of the M4 is UP or DOWN.)
        
        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        Nothing.
        """
        ret = self._state()
        print('THE PRESENT STATE OF THE M4\n'
              '[{0}]'.format(ret))
        return ret

    def close_box(self):
        """        
        DESCRIPTION
        ================
        This function close the board connection.
        
        ARGUMENTS
        ================
        Nothing.
        
        RETURNS
        ================
        Nothing.
        """
        self.mtnc.close_board()
        return

    def pub_status(self):
        pub = rospy.Publisher('status_m4',Status_m4_msg, queue_size=10, latch = True)
        msg = Status_m4_msg()

        while not rospy.is_shutdown():
            pos = self.query_position
            msg.m4_position = pos
            pub.publish(msg)
            rospy.loginfo(self.position)
            time.sleep(0.5)
        return


if __name__ == '__main__':
    m4 = m4()
    rospy.init_node('m4_controller')
    rospy.loginfo('waiting publish M4')
    m4.start_thread()
    sub = rospy.Subscriber('m4', String, m4.move)
    rospy.spin()
