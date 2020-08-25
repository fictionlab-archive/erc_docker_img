#!/bin/bash -e

source /opt/ros/melodic/setup.bash

echo "source /opt/ros/melodic/setup.bash" >> /etc/bash.bashrc
echo "ROS_IP=${ROS_IP}" >> /etc/environment
echo "ROS_MASTER_URI=${ROS_MASTER_URI}" >> /etc/environment

python /root/.local/lib/python2.7/site-packages/freedomrobotics/agent.py