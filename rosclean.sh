#!/bin/sh
source /opt/ros/kinetic/setup.bash
source $HOME/ros/devel/setup.bash
date >>/tmp/rosclean.log 
rosclean check >>/tmp/rosclean.log 2>>/tmp/rosclean_error.log
rosclean purge -y >>/tmp/rosclean.log 2>>/tmp/rosclean_error.log
rosclean check >>/tmp/rosclean.log 2>>/tmp/rosclean_error.log

