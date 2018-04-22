#!/bin/bash

export MASTER_IP=10.42.0.1
export ROS_MASTER_URI=http://$MASTER_IP:11311
export ROS_IP=`ip route get $MASTER_IP | awk '{print $NF; exit}'`
