#!/bin/bash

python3 -m http.server 8080 --directory ./src/remote_control &
web_pid=$!

ros2 launch rosbridge_server rosbridge_websocket_launch.xml bind_address:=192.168.31.171 &
ros_pid=$!

cleanup(){
    kill -9 "$web_pid" 2>/dev/null
    kill -9 "$ros_pid" 2>/dev/null
    exit 0
}
trap cleanup EXIT
wait -n $web_pid $ros_pid