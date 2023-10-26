#! /bin/bash

source /root/.bashrc;
python3 /root/catkin_ws/src/my_navigation_configs/python/singlerun.py $1 2> >(grep -v -E 'TF_REPEATED|buffer_core.cpp|^$');

