import os, datetime, linecache
import libMotor as lib
import NASCORX_System.device.CPZ7415V as CPZ7415V
import sys
import rospy
import time
import threading

from std_msgs.msg import String 
from necst.msg import Status_hot_msg

class abs(object):
    """
    DESCRIPTION
    ================
    This class controls the chopper.

    ARGUMENTS
    ================
    1. board: chopper class is oprated by board of 'CPZ7415Vb'
        Type: string
        board: 'CZP7415Vb'
    """

    def __init__(self):
        dev = 2 # CPZ7415Vb
        ROT = 9
        axis = lib.MOTOR_ZAXIS
        log_file = chopper_log.txt
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
        This function logs the chopper state means HOT or COLD in the logfile.
        
        ARGUMENTS
        ================
        1. con: chopper state
            Type: str
            Default: HOT

        RETURNS
        ================
        Nothing.
        """
        da = ['{0:%Y-%m-%d %H:%M:%S}'.format(datatime.datetime.now), ' ' , '{0}\n'.format(con)]
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
        This function logs the chopper state means HOT or COLD in the logfile.
        
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
        This function moves the chopper to HOT.
        
        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        Nothing.
        """
        if req.data == 'HOT':
            if self._state.split()[2] = 'COLD':
                self.mtnc.move_ccw(strk=[ROT,1,1,1], axis=axis)
                self._log(con='HOT')
                self.close_box()
                return
            else:
                print('!!!!ERROR!!!!\n'
                      'Chopper has already moved to HOT.')

        if req.data == 'COLD': 
            if self._state.split()[2] = 'HOT':
                self.mtnc.move_cw(strk=[ROT,1,1,1], axis=axis)
                self._log(con='COLD')
                self.close_box()
                return
            else:
                print('!!!!ERROR!!!!\n'
                      'Chopper has already moved to COLD.')
                  
            
    def query_position(self):
        """        
        DESCRIPTION
        ================
        This function queries the chopper position.
        (The M4 positino means state of the chopper is HOT or COLD.)
        
        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        Nothing.
        """
        ret = self._state()
        print(ret)
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
        pub = rospy.Publisher('status_hot', Status_hot_msg, queue_size=10, latch = True)
        msg = Status_hot_msg()

        while not rospy.is_shutdown():
            pos = self.query_position()
            print(pos)
            msg.hot_position = pos
            pub.publish(msg)
            rospy.loginfo(pos)
            time.sleep(0.5)
        return

if __name__ == '__main__':
    abs = abs()
    rospy.init_node('abs_controller')
    rospy.loginfo('waiting publish abs')
    abs.start_thread()
    sub = rospy.Subscriber('hot', String, abs.move)
    rospy.spin()


