#! /bin/bash

#mavproxy.py --master=127.0.0.1:14550 --out=127.0.0.1:14551 --out=192.168.205.102:14552
mavproxy.py --master=127.0.0.1:14650 --out=127.0.0.1:14550 --out=192.168.207.85:14552

#export ROS_IP=192.168.0.27
#roscore
#sleep 5
#roslaunch rplidar_ros rplidar.launch
#roslaunch hector_slam_launch tutorial.launch
#rosrun mavros mavros_node _fcu_url:=udp://:14650@ _gcs_url:=udp://:14551@192.168.0.9:14552
