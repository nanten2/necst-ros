import rospy
#import pyinterface
from necst.msg import Test_board_msg


class board(object):

    def __init__(self):
        #self.dio = pyinterface.create_gpg2000(3)
        #rospy.init_node("board")
        pass

    def out_word(self,no,value):
        #rospy.init_node("board")
        pub = rospy.Publisher("status_board", Test_board_msg, queue_size=10, latch=True)
        msg = Test_board_msg()
        #self.dio.ctrl.out_word(no,value)
        if no == "FBIDIO_OUT1_16":
            rospy.loginfo("az_motor")
            rospy.loginfo(value)
            msg.azel = "az"
            msg.vel_az = value
        elif no == "FBIDIO_OUT17_32":
            rospy.loginfo("el_motor")
            rospy.loginfo(value)
            msg.azel = "el"
            msg.vel_el = value
        else:
            rospy.loginfo("#############################")
        pub.publish(msg)
        return
            

    def in_byte(self,no):
        #return self.dio.ctrl.in_byte(no)
        #print("name : ", no)
        return 


