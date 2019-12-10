#!/bin/bash
path="~/ros/src/necst/obs_scripts"
maxtimes=20
if [ $maxtimes -lt $1 ] ; then
    echo "Too much repeat times(>20)"
else
    echo "observation Repeat $1 times"
    for data in `seq $1`
    do
	`$path/cross_point_xffts2.py --obsfile line_cross_IRC10216.obs`
    done
fi
