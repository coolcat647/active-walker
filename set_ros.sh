#!/usr/bin/env bash
COLOR_RED='\033[0;31m'
COLOR_GREEN='\033[0;32m'
COLOR_YELLOW='\033[0;33m'
COLOR_NC='\033[0m'

# Set ROS_MASTER_URI to walker
export ROS_MASTER_URI=http://10.42.0.1:11311

# Set ROS_IP with pofix "10.42.0."
export ROS_IP=$(hostname -I |grep -oP '10.42.0.{4}' |cut -d ' ' -f 1)
if [ -z "${ROS_IP}" ]; then
    export ROS_IP=$(hostname -I |grep -oP '10.42.0.{3}' |cut -d ' ' -f 1)
fi
if [ -z "${ROS_IP}" ]; then
    export ROS_IP=$(hostname -I |grep -oP '10.42.0.{2}' |cut -d ' ' -f 1)
fi

source $(find $(pwd) -iname 'setup.sh' |grep "catkin_ws/devel")

echo -e "${COLOR_GREEN}ROS_MASTER_URI= ${ROS_MASTER_URI}${COLOR_NC}"
echo -e "${COLOR_GREEN}ROS_IP= ${ROS_IP}${COLOR_NC}"