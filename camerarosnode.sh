#!/bin/bash
eval echo "[cameraRosNode] starting ... " $toStartlog
scriptPath=/home/unitree/Unitree/autostart/
export SUDO_ASKPASS=${scriptPath}passwd.sh
interfaces_lines=$(cat /etc/network/interfaces | wc -l)
if [ $interfaces_lines -gt 3 ]; then
    sleep 0.5
    cd cameraRosNode
    ./startNode.sh
    cd ../
fi
