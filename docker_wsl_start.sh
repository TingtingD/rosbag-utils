{
    docker kill rosbag-util || true
    docker rm rosbag-util
} &> /dev/null

if [ -z "$1" ]; then
    echo -e "To change dataset folder, run \e[33m./docker_start.sh /your/folder/here\e[0m"
fi

FOLDER="${1:-/}"
echo -e "Starting rosbag-utils with /data folder: \e[32m${FOLDER}\e[0m"

# (sleep 1 && xdg-open http://127.0.0.1:8000 > /dev/null 2>&1) &

docker run \
-it \
--name rosbag-util \
-v $(readlink -f "./."):/root/rosbag-utils \
-v "$FOLDER:/data" \
-p 8000-8001:8000-8001 \
-e "DISPLAY" \
-e "QT_X11_NO_MITSHM=1" \
-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--privileged \
rosbag-utils:latest
