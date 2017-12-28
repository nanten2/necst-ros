export ROS_MASTER_URI=http://172.20.0.14:11311
export ROS_IP=172.20.0.14
source /opt/ros/kinetic/setup.bash
source /home/amigos/ros/devel/setup.bash

python2 oneshot_ch.py
