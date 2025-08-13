#!/bin/bash

script_dir=$(cd "$(dirname "$0")" && pwd)
cd "$script_dir" || exit 1

get_port() {
    python3 -c "import json, sys; data = json.load(open('config.json')); print(data['$1'])"
}

HTTP_PORT=$(get_port "http_port")
ROSBRIDGE_PORT=$(get_port "rosbridge_port")

sudo bash -c '
    PORT='"$HTTP_PORT"'
    PID_LSOF=$(lsof -t -i :$PORT 2>/dev/null)
    PID_NETSTAT=$(netstat -tulpn 2>/dev/null | grep ":$PORT " | awk "{print \$7}" | cut -d/ -f1)
    ALL_PIDS=$(echo "$PID_LSOF $PID_NETSTAT" | tr " " "\n" | sort -u | grep -v "^$")
    if [ -n "$ALL_PIDS" ]; then
        echo "终止占用端口 $PORT 的进程："
        for PID in $ALL_PIDS; do
            ps -p $PID -o pid,user,comm,cmd --no-headers 2>/dev/null
            kill -15 $PID 2>/dev/null; sleep 0.5
            ps -p $PID >/dev/null && kill -9 $PID
        done
    fi
'

sudo bash -c '
    PORT='"$ROSBRIDGE_PORT"'
    PID_LSOF=$(lsof -t -i :$PORT 2>/dev/null)
    PID_NETSTAT=$(netstat -tulpn 2>/dev/null | grep ":$PORT " | awk "{print \$7}" | cut -d/ -f1)
    ALL_PIDS=$(echo "$PID_LSOF $PID_NETSTAT" | tr " " "\n" | sort -u | grep -v "^$")
    if [ -n "$ALL_PIDS" ]; then
        echo "终止占用端口 $PORT 的进程："
        for PID in $ALL_PIDS; do
            ps -p $PID -o pid,user,comm,cmd --no-headers 2>/dev/null
            kill -15 $PID 2>/dev/null; sleep 0.5
            ps -p $PID >/dev/null && kill -9 $PID
        done
    fi
'

python3 -m http.server "$HTTP_PORT" &
web_pid=$!

ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:="$ROSBRIDGE_PORT" &
ros_pid=$!

cleanup(){
    kill -9 "$web_pid" 2>/dev/null
    kill -9 "$ros_pid" 2>/dev/null
    exit 0
}
trap cleanup EXIT
wait -n $web_pid $ros_pid