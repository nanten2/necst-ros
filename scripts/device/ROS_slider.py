import os, datetime, linecache
import libMotor as lib
import NASCORX_System.device.CPZ7415V as CPZ7415V

class slider(object):
    """
    DESCRIPTION
    ================
    This class controls the slider.

    ARGUMENTS
    ================
    1. board: name of the motion controller registered in the IP_table.
        Type: string
        Default: 'CZP7415Va'
    2. device_table: file path of the IP table
        Type: string
        Default: '/home/amigos/NASCORX-master/base/IP_table_115.txt'
    """

    def __init__(self, board='CPZ7415Va', device_table='/home/amigos/NASCORX-master/base/IP_table_115.txt'):
        self.board = board
        self.device_table = device_table
        self.nboard = self._board_search_(device=self.board)
        self.mtnc = CPZ7415V.cpz7415v(dev=self.nboard[1])

    def _board_search_(self, device):
        f = open(self.device_table, 'r')
        for line in f:
            dev = line.strip().split(',')
            if device in dev:
                info1 = int(dev[1].strip())
                break
            else:
                pass
        f.close()
        ret = [device, info1]
        return ret

    def slide_cw(self, strk=1, axis=lib.MOTOR_XAXIS):
        if self.board == 'CPZ7415Va' and axis == X:
            axis = lib.MOTOR_XAXIS_1
        elif self.board == 'CPZ7415Vb' and axis == X:
            axis = lib.MOTOR_XAXIS_2
        else: pass
        if 0 < strk < 150:
            self.mtnc.move_cw(strk=[strk,1,1,1], axis=axis)
            self.close_box()
            return
        else:
            print('!!!!ERROR!!!!\n'
                  'Please input invalid parameter.\n'
                  '0 < strk < 150')

        if self.board == 'CPZ7415Va' and axis == X:
            axis = lib.MOTOR_XAXIS_1
        elif self.board == 'CPZ7415Vb' and axis == X:
            axis = lib.MOTOR_XAXIS_2
        else: pass
        if 0 < strk < 150:
            self.mtnc.move_ccw(strk=[strk,1,1,1], axis=axis)
            self.close_box()
            return
        else:
            print('!!!!ERROR!!!!\n'
                  'Please input invalid parameter.\n'
                  '0 < strk < 150')


    def return_org(self, out=lib.RETURN_ORIGIN_XAXIS, req):
        """        
        DESCRIPTION
        ================
        This function let the slider return origin.
        
        ARGUMENTS
        ================
        1. out: output pin
            Type: int(bit)
            Derault: RETURN_ORIGIN_XAXIS

        RETURNS
        ================
        Nothing.
        """
        self.mtnc.output_do(out=out)
        return

    def reset_alarm(self):
        """        
        DESCRIPTION
        ================
        This function resets alarm of 
        all motion driver controlled by the motion controller board.
        
        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        Nothing.
        """
        out = INITIAL_STATE
        self.mtnc.output_do(out=out)
        return

    def query_position(self, pos=lib.MOTOR_XAXIS_POS):
        """        
        DESCRIPTION
        ================
        This function queries the slider position.
        
        ARGUMENTS
        ================
        1. pos: select the slider
            Type: int
            Default: MOTOR_XAXIS_1_POS
        
        RETURNS
        ================
        1. position [mm]
            Type: fault
            About: position is destance from the slider origin.
        """
        ret = self.query_position()
        position = ret[pos]*0.01
        return position

    def close_board(self):
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
        pub = rospy.Publisher('status_slider', Status_slider_msg, queue_size=10, latch = True)
        msg = Status_slider_msg()

        while not rospy.is_shutdown():
            pos = self.query_position()
            print(pos)
            msg.slider_position = pos
            pub.publish(msg)
            rospy.loginfo(pos)
            time.sleep(0.5)
        return

if __name__ == '__main__':
    sli = slider()
    rospy.init_node('slider')
    rospy.loginfo('waiting publish slider')
    sli.start_thread()
    sub = rospy.Subscriber('sli', String, sli.move)
    sub = rospy.Subscriber('sli_org', String, sli.return_org)
    rospy.spin()
