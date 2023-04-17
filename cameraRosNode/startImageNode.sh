#!/bin/bash
localIP=`cat /etc/network/interfaces | grep "address" | awk '{print $2}'`
export ROS_MASTER_URI=http://192.168.123.161:11311
export ROS_IP=$localIP
source ./devel/setup.bash
patchlines=$(cat patch | wc -l)

sudo ntpdate -d 192.168.123.161 > offset
timeoffset=$(cat offset | grep "filter offset:" | awk '{print $3}')
rm offset

if [ "${patchlines}" == "1" ]; then
    camNum=$(cat patch | head -n 1 | awk '{print $2}')
    rosrun unitree_camera image_publish _run_config_file:=./src/unitree_camera/config/stereo_camera_config.yaml camera${camNum} ${timeoffset} &
    sleep 5
elif [ "${patchlines}" == "2" ]; then
    camNum1=$(cat patch | head -n 1 | awk '{print $2}')
    camNum2=$(cat patch | tail -n 1 | awk '{print $2}')
    if [ "${camNum1}" == "2" ]; then
        rosrun unitree_camera example_point ./src/unitree_camera/config/stereo_camera_config.yaml &
    else
        rosrun unitree_camera image_publish _run_config_file:=./src/unitree_camera/config/stereo_camera_config.yaml camera${camNum1} ${timeoffset}  &
    fi
    sleep 5
    if [ "${camNum2}" == "2" ]; then
        rosrun unitree_camera example_point ./src/unitree_camera/config/stereo_camera_config1.yaml &
    else
        rosrun unitree_camera image_publish _run_config_file:=./src/unitree_camera/config/stereo_camera_config1.yaml camera${camNum2} ${timeoffset}  &
    fi
    sleep 5
fi

