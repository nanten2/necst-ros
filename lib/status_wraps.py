import time
import rospy
from std_msgs.msg import String
from necst.msg import Status_node_msg

def deco(node_name="NONE", topic_name="NONE", frame_name="NONE"):
    """node_name=os.path.basename(__file__)"""
    def _deco(func):
        pub = rospy.Publisher("status_node", Status_node_msg, queue_size=1, latch=True)
        import functools
        @functools.wraps(func)
        def wrapper(*args,**kwargs):
            pub.publish(node_name, topic_name, frame_name)
            time.sleep(0.001)
            func(*args,**kwargs)
            #pub.publish(node_name, func_name, "end")
        return wrapper
    return _deco

"""
@deco
def test2():
    print('Hello Decorator')

test2()
"""
